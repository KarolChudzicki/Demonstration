import cv2 as cv #opencv-python
import time
import numpy as np
import imutils #imutils
import math
import calibration as calib
import sliders_hsv


camera_matrix = np.array([
    [701.43142873, 0, 340.89422739],
    [0, 700.18225122, 231.27284128],
    [0, 0, 1]
])

dist_coeffs = np.array([[-4.40734319e-01, 3.89166568e-01, -6.93778517e-05, 2.41658581e-03, -2.90304795e-01]])


rvecs = np.array([
    [0.47038998, -0.23224266, -1.36597417],
    [-0.07981019, -0.16647916, 1.61505969],
    [-0.17148689, -0.48068229, 1.50307489],
    [-0.27495634, -0.34021001, 1.59257956],
    [-0.01823973, -0.27598613, 1.5211003],
    [0.31829143, -0.04453068, -1.10016429],
    [-0.21668051, -0.30041154, 0.96543265],
    [-0.26247403, 0.03388248, 1.55473485],
    [0.17765802, 0.09588896, 1.44155328],
    [0.39340981, -0.27157359, -1.39048863],
    [-0.07318423, -0.1732357, 1.48705966],
    [0.08372568, -0.47871739, 1.46270206],
    [-0.13067318, 0.17032886, 1.55662348],
    [-0.04957571, -0.2199913, 1.43057739],
    [0.04117001, -0.10908014, 0.92428516],
    [0.14109622, -0.52752174, -1.44859329],
    [0.33300622, -0.46388028, -1.1810463],
    [-0.08275248, -0.38454006, 1.49043579],
    [-0.18602322, -0.27678638, 1.10457917],
    [0.40760421, -0.06575685, -0.99286398]
])


tvecs = np.array([
    [[-4.16866643], [2.56145043], [23.55532557]],
    [[9.92346982], [0.47658607], [25.28933017]],
    [[-2.20838949], [-7.91495415], [29.14511795]],
    [[-1.4544579], [1.11469722], [30.59347155]],
    [[9.91888708], [-6.9861772], [26.46215193]],
    [[-6.94444007], [1.14183611], [25.40027094]],
    [[2.76274708], [-6.37237717], [25.27577111]],
    [[4.31675679], [-3.66666617], [27.87477077]],
    [[4.24090864], [-3.35521445], [30.15415769]],
    [[-5.28462817], [2.63452814], [23.51916734]],
    [[4.03681646], [-3.40591733], [26.65658219]],
    [[3.63015677], [-3.6804039], [29.34216628]],
    [[5.0664787], [-4.01336519], [18.52521328]],
    [[5.98023528], [-4.5393692], [29.46769702]],
    [[2.61681378], [-5.3257859], [29.71997933]],
    [[-4.5112023], [2.47364792], [29.94940236]],
    [[-3.71141982], [1.23030042], [23.98786941]],
    [[5.53142058], [-7.20677587], [26.92446391]],
    [[2.56458315], [-5.54783017], [27.04624868]],
    [[-8.02427971], [0.51851539], [25.71631894]]
])


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


sliders_hsv.init(50,50,50,0)

while True:
    (ret, frame) = cap.read()
    if ret:
        frame_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        frame_gray_main = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        
        
        new_camera_matrix, roi = cv.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (1920, 1080), 1, (1920, 1080))

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
            if area > 100:
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
                corner_cnt = 0
                for corner in approx_poly:
                    x, y = corner.ravel()
                    cv.circle(frame, (x, y), 5, (0, 0, 255), -1)  # Corners in red
                    cv.putText(frame, str(corner_cnt), (x + 10,y + 10), cv.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2, cv.LINE_AA)
                    corner_cnt += 1
                    
                
                image_points = np.array([point[0] for point in approx_poly], dtype=np.float32)
                #success, rotation_vector, translation_vector = cv.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)
                #rotation_matrix, _ = cv.Rodrigues(rotation_vector)
                
                #cube_center_object_space = np.mean(object_points, axis=0)
                #cube_center_camera_space = np.dot(rotation_matrix, cube_center_object_space.reshape(-1, 1)) + translation_vector

                # Print the 3D coordinates of the cube's center
                #print("Cube Center in 3D Space (relative to camera):", cube_center_camera_space.ravel())
                
        
                

                # Draw the approximated polygon
                cv.drawContours(frame, [approx_poly], -1, (255, 0, 0), 2)  # Draw the approximated polygon in red
                
                
                
                               
                rect = cv.minAreaRect(c)  # rect contains (center, (width, height), angle)
                
        
                     
                           
        #cv.circle(frame, (x//2, y//2), 5, (255,255,255), 2)
        cv.imshow('img1',frame)
        cv.imshow('esges',frame_thresh)
    else:
        print("Camera error")
        break
    
    if cv.waitKey(1) & 0xFF == ord('q'):
        break


