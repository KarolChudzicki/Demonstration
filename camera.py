import cv2 as cv #opencv-python
import time
import numpy as np
import imutils #imutils
import math
import calibration as calib
import sliders_hsv

ql = 0.001
cap = cv.VideoCapture(0) 
previous_points = None
average1 = []
moving_avg_len1 = 10
average2 = []
moving_avg_len2 = 10
angle = 0
sum_loop = 0


# Get camera calibration parameters
calibration_result = calib.run()

if calibration_result is None:
    print("Calibration failed or no chessboard corners detected.")
    exit()
else:
    camera_matrix, dist_coeffs, rvecs, tvecs = calibration_result

sliders_hsv.init(50,50,50,5)

while True:
    (ret, frame) = cap.read()
    if ret:
        frame_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        frame_gray_main = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        
        # Draw stuff for assistance
        height, width = frame.shape[:2]
        center_x = width // 2
        center_y = height // 2
        
        new_camera_matrix, roi = cv.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (width, height), 1, (width, height))

        # Undistort the image
        undistorted_img = cv.undistort(frame, camera_matrix, dist_coeffs, None, new_camera_matrix)

        
        # Crop the image to the valid region of interest
        x, y, w, h = roi
        undistorted_img = undistorted_img[y:y+h, x:x+w]
        
        #image = np.copy(frame)

        frame, result, frame_thresh = sliders_hsv.sliders(undistorted_img)
        
        

        edges = cv.Canny(frame_thresh, 50, 200)
            
        contours, _ = cv.findContours(edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
        
        for c in contours:
            #peri = cv.arcLength(c, True)
            area = cv.contourArea(c)
            if area > 1000:
                (x,y),radius = cv.minEnclosingCircle(c)
                center = (int(x),int(y))
                cv.circle(frame,center,2,(255,0,0),2)

                
                hull = cv.convexHull(c)

                # Draw the convex hull around the cube
                cv.drawContours(frame, [hull], -1, (0, 255, 0), 2)  # Draw the convex hull in green
                
                # Approximate the convex hull to simplify it to a polygon with straight lines
                epsilon = 0.01 * cv.arcLength(hull, True)  # Precision of approximation
                approx_poly = cv.approxPolyDP(hull, epsilon, True)

                # Draw the approximated polygon
                cv.drawContours(frame, [approx_poly], -1, (255, 0, 0), 2)  # Draw the approximated polygon in red
                
                               
                rect = cv.minAreaRect(c)  # rect contains (center, (width, height), angle)
        
                '''
                # Unpack the rectangle data
                center, size, angle_rect = rect
                area_rect = size[0] * size[1]
                print(area_rect/area)
                # Draw the bounding box for visualization
                box = cv.boxPoints(rect)  # Get 4 corners of the rectangle
                box = np.int0(box)        # Convert to integer points
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
                '''                      
                           
        cv.circle(frame, (center_x, center_y), 5, (255,255,255), 2)
        cv.imshow('img1',frame)
        cv.imshow('esges',edges)
    else:
        print("Camera error")
        break
    
    if cv.waitKey(1) & 0xFF == ord('q'):
        break




