import cv2 as cv
import time
import numpy as np
import imutils
import math

#KERNELS AND VARIABLES
x = 0
k_size = 5
kernel = np.ones((k_size,k_size))
kernel = kernel/(k_size*k_size)

#THRESHOLD
min_value = 35
max_value = 255

cap = cv.VideoCapture(1)
while True:
    (ret, frame) = cap.read()
    if ret:
        frame_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        # HSV ranges
        red_low1 = np.array([0, 150, 100])
        red_low2 = np.array([10, 255, 255])

        red_high1 = np.array([230, 150, 100])
        red_high2 = np.array([255, 255, 255])

        # Create a mask for red color
        mask1 = cv.inRange(frame_hsv, red_low1, red_low2)
        mask2 = cv.inRange(frame_hsv, red_high1, red_high2)

        mask = mask1 | mask2

        # Apply the mask to the original image to extract red regions
        result = cv.bitwise_and(frame, frame, mask=mask)
        
        frame_gray = cv.cvtColor(result, cv.COLOR_BGR2GRAY)
        frame_gray = cv.filter2D(frame_gray, -1, kernel)
        
        

        thresh = cv.threshold(frame_gray, min_value, max_value, cv.THRESH_BINARY)[1]
        
        cnts = cv.findContours(thresh.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        for c in cnts:
            area = cv.contourArea(c)            
            if area > 20000:
                # limit contour to quadrilateral
                peri = cv.arcLength(c, True)
                corners = cv.approxPolyDP(c, 0.04 * peri, True)

                # draw quadrilateral on input image from detected corners
                cv.polylines(frame, [corners], True, (255,255,0), 1, cv.LINE_AA)
                
                # getting two points
                P1 = corners[0].ravel()
                P2 = corners[1].ravel()
                
                cv.circle(frame,corners[0].ravel(), 5, (255,255,0), -1)
                cv.circle(frame,corners[1].ravel(), 5, (255,255,0), -1)
                
                d1 = math.sqrt(pow(P1[0]-P2[0],2)+pow(P1[1]-P2[1],2))
                d2 = P1[1] - P2[1]
                angle = -math.asin(d2/d1)
                angle = round(math.degrees(angle),2)
                
                
                #cv.drawContours(frame, [c], -1, (255, 255, 0), 2)
                cv.putText(frame, str(angle) + "deg", (100,100), cv.FONT_HERSHEY_COMPLEX, 1, (255,0,0), 2)
                
        cv.imshow('img1', frame)
    else:
        print("Camera error")
        break
    
    if cv.waitKey(1) & 0xFF == ord('q'):
        break