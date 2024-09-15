import cv2 as cv #opencv-python
import time
import numpy as np
import imutils #imutils
import math

ql = 0.001
cap = cv.VideoCapture(0) 
previous_points = None
average1 = []
moving_avg_len1 = 10
average2 = []
moving_avg_len2 = 10
angle = 0
sum_loop = 0
while True:
    (ret, frame) = cap.read()
    if ret:
        frame_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        frame_gray_main = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        image = np.copy(frame)

        # Draw stuff for assistance
        height, width = frame.shape[:2]
        center_x = width // 2
        center_y = height // 2
        cv.rectangle(frame, (center_x + 100, center_y + 100), (center_x - 100, center_y - 100), (255,255,255), 1)

        cv.circle(frame, (center_x, center_y), 5, (255,255,255), 2)

        # HSV ranges
        red_lower1 = np.array([0, 150, 100])
        red_upper1 = np.array([10, 255, 255])

        red_lower2 = np.array([170, 150, 100])
        red_upper2 = np.array([180, 255, 255])

        green_lower = np.array([35, 100, 100])
        green_upper = np.array([85, 255, 255])

        # Create masks for each color
        mask1_red = cv.inRange(frame_hsv, red_lower1, red_upper1)
        mask2_red = cv.inRange(frame_hsv, red_lower2, red_upper2)

        mask_green = cv.inRange(frame_hsv, green_lower, green_upper)


        mask_red = mask1_red | mask2_red
        mask = mask_green

        # Apply the mask to the original image to extract red regions
        result = cv.bitwise_and(frame, frame, mask=mask)
        
        frame_gray = cv.cvtColor(result, cv.COLOR_BGR2GRAY)
        #Adaptive Histogram Equalization
        #Enhances contrast, which can be especially useful if the image has varying lighting conditions.
        #clahe = cv.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        #frame_gray = clahe.apply(frame_gray)
        frame_blur = cv.GaussianBlur(frame_gray, (9,9), 0)
        _, frame_blur = cv.threshold(frame_blur, 0, 255, cv.THRESH_BINARY)

        edges = cv.Canny(frame_blur, 50, 200)
            
        contours, _ = cv.findContours(edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        
        for c in contours:
            #peri = cv.arcLength(c, True)
            area = cv.contourArea(c)
            if area > 0:
                (x,y),radius = cv.minEnclosingCircle(c)
                center = (int(x),int(y))
                cv.circle(frame,center,2,(255,0,0),2)
                #radius = int(radius)
                #cv.circle(frame,center,radius,(0,255,0),2)
                
                hull = cv.convexHull(c)

                # Draw the convex hull around the cube
                cv.drawContours(frame, [hull], -1, (0, 255, 0), 2)  # Draw the convex hull in green
                
                # Approximate the convex hull to simplify it to a polygon with straight lines
                epsilon = 0.02 * cv.arcLength(hull, True)  # Precision of approximation
                approx_poly = cv.approxPolyDP(hull, epsilon, True)

                # Draw the approximated polygon
                cv.drawContours(frame, [approx_poly], -1, (255, 0, 0), 2)  # Draw the approximated polygon in red
                
                               
                rect = cv.minAreaRect(c)  # rect contains (center, (width, height), angle)
        
                # Unpack the rectangle data
                center, size, angle_rect = rect
                area_rect = size[0] * size[1]
                print(area_rect/area)
                # Draw the bounding box for visualization
                box = cv.boxPoints(rect)  # Get 4 corners of the rectangle
                box = np.int0(box)        # Convert to integer points
                #cv.drawContours(frame, [box], 0, (0, 255, 0), 2)
                
                cv.putText(frame, str(round(size[0]/size[1],4)), (100,100), cv.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)
                
                if radius > 0:
                    circle_area = math.pi * radius*radius
                    proportion = area/circle_area
                    if len(average1) < moving_avg_len1:
                        average1.append(proportion)
                        average2.append(angle)
                    else:
                        average1.pop(0)
                        average1.append(proportion)
                        average2.pop(0)
                        average2.append(angle)
                            

                    angle = 45 * (average1[len(average1)-1] - 0.68)/(0.81-0.68)
                        
                    actual_angle = round(sum(average2)/len(average2)/5)*5
                    #print(average1[len(average1)-1])
                    #print(actual_angle)
                    
                
                    
                    
                            
            
                

                

        cv.imshow('img1',edges)
    else:
        print("Camera error")
        break
    
    if cv.waitKey(1) & 0xFF == ord('q'):
        break




