import cv2 as cv #opencv-python
import time
import numpy as np
import imutils #imutils
import math
import calibration as calib
import sliders_hsv


# Parameters
ql = 0.001
cap = cv.VideoCapture(0) 
previous_points = None
average1 = []
moving_avg_len1 = 10
average2 = []
moving_avg_len2 = 10
angle = 0
sum_loop = 0

L = 31  # The size of the cube (known)
object_points = np.array([
    [0, 0, 0],  # Corner 1
    [L, 0, 0],  # Corner 2
    [0, L, 0],  # Corner 3
    [L, 0, L],  # Corner 4
    [L, L, L],  # Corner 5
    [0, L, L]   # Corner 6
], dtype=np.float32)

# Get camera calibration parameters
calibration_result = calib.run()

if calibration_result is None:
    print("Calibration failed or no chessboard corners detected.")
    exit()
else:
    camera_matrix, dist_coeffs, rvecs, tvecs = calibration_result

sliders_hsv.init(50,50,50,0)

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
        #x, y, w, h = roi
        #undistorted_img = undistorted_img[y:y+h, x:x+w]
        
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
                
                # Make sure that the algorith detects exactly 6 corners
                while len(approx_poly) != 6:
                    approx_poly = cv.approxPolyDP(hull, epsilon, True)
                    if len(approx_poly) > 6:
                        epsilon *= 1.05
                    else:
                        epsilon *= 0.95
                for corner in approx_poly:
                    x, y = corner.ravel()
                    cv.circle(frame, (x, y), 5, (0, 0, 255), -1)  # Corners in red
                    
                
                image_points = np.array([point[0] for point in approx_poly], dtype=np.float32)
                success, rotation_vector, translation_vector = cv.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)
                rotation_matrix, _ = cv.Rodrigues(rotation_vector)
                
                cube_center_object_space = np.mean(object_points, axis=0)
                cube_center_camera_space = np.dot(rotation_matrix, cube_center_object_space.reshape(-1, 1)) + translation_vector

                # Print the 3D coordinates of the cube's center
                print("Cube Center in 3D Space (relative to camera):", cube_center_camera_space.ravel())
                
        
                

                # Draw the approximated polygon
                cv.drawContours(frame, [approx_poly], -1, (255, 0, 0), 2)  # Draw the approximated polygon in red
                
                
                
                               
                rect = cv.minAreaRect(c)  # rect contains (center, (width, height), angle)
                
        
                     
                           
        cv.circle(frame, (center_x, center_y), 5, (255,255,255), 2)
        cv.imshow('img1',frame)
        cv.imshow('esges',edges)
    else:
        print("Camera error")
        break
    
    if cv.waitKey(1) & 0xFF == ord('q'):
        break




'''
        '''