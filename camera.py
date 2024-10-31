import cv2 as cv #opencv-python
import time
import numpy as np
import imutils #imutils
import math


camera_matrix = np.array([
    [701.43142873, 0, 340.89422739],
    [0, 700.18225122, 231.27284128],
    [0, 0, 1]
])

distortion_coeffs = np.array([[-4.40734319e-01, 3.89166568e-01, -6.93778517e-05, 2.41658581e-03, -2.90304795e-01]])


rotation_vectors = np.array([
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


translation_vectors = np.array([
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
aspect_ratio = 1
angle = 0
sum_loop = 0
check_for_outliers = False
cubeTopArea = 0
saturationThresholdMask = 170
cx = None
approx_stacked = []

width = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))

xyz = np.empty((0,3))
angles = 0
coordinates = 0

L = 30  # The size of the cube (known)
P = L*math.sqrt(2)/2

cube_points = np.array([
    [-L/2, -L/2, 0],  # Corner 1
    [-L/2, L/2, 0],  # Corner 2
    [L/2, L/2, 0],  # Corner 3
    [L/2, -L/2, 0]   # Corner 4
], dtype=np.float32)


def coords(points, approx, frame):
    image_points = np.array([point[0] for point in approx], dtype=np.float32)
    # Maybe use cv.SOLVEPNP_P3P, but it only works for 4 points cv.SOLVEPNP_EPNP
    if len(approx) == 4:
        success, rotation_vector, translation_vector = cv.solvePnP(points, image_points, camera_matrix, distortion_coeffs, flags=cv.SOLVEPNP_P3P)
    else:
        print("Incorrect number of corners")
    
    if success:
        rotation_matrix, _ = cv.Rodrigues(rotation_vector)
        cube_center_object_space = np.mean(points, axis=0)
        cube_center_camera_space = np.dot(rotation_matrix, cube_center_object_space.reshape(-1, 1)) + translation_vector

        coordinates_camera_space = np.round(cube_center_camera_space.ravel()[:3]).astype(int)

        # Yaw, pitch, roll
        try:
            U, _, Vt = np.linalg.svd(rotation_matrix)
        
            # Compute the new rotation matrix
            # possible alternative is to use Singular Value Decomposition (SVD)
            # on the rotation matrix to extract the rotation angles directly.
            # This method is less susceptible to some of the numerical instability issues that can arise from Euler angle calculations. 
            R = np.dot(U, Vt)
            yaw = math.atan2(R[1, 0], R[0, 0])
            pitch = math.atan2(-R[2, 0], math.sqrt(R[2, 1]**2 + R[2, 2]**2))
            roll = math.atan2(R[2, 1], R[2, 2])

            yaw = round(np.degrees(yaw))
            pitch = round(np.degrees(pitch))
            roll = round(np.degrees(roll))           
            

            # Project the 3D points to 2D image points using the pose (rvec, tvec)
            img_points, _ = cv.projectPoints(points, rotation_vector, translation_vector, camera_matrix, distortion_coeffs)

            # Visualize the projected points on the image (assuming 'frame' is your video frame)
            for point in img_points:
                x, y = point.ravel()
                cv.circle(frame, (int(x), int(y)), 5, (255, 255, 0), -1)  # Draw green dots
            
            
        except Exception as e:
            print(f"Error in angle calculation: {e}")
            return None

        coordinates = np.round(coordinates_camera_space.ravel()[:3]).astype(int).tolist()
        angles = [yaw, pitch, roll]
       
       #Remove roll
        coordinates.pop(2)
        return coordinates, angles
    else:
        print("Solve PNP error")
        return 0,0

def sort_points(points, number):
    
    if number == 4:
        sorted_points_by_y = sorted(points, key=lambda point: point[0][1])
        sorted_points_by_x = sorted(points, key=lambda point: point[0][0])
        
        return [sorted_points_by_y[0], sorted_points_by_x[0], sorted_points_by_y[-1], sorted_points_by_x[-1]]
    
    else:
        print("Invalid number of points")
        return None



#================================ MAIN LOOP ================================
while True:
    (ret, frame) = cap.read()
    fps = cap.get(cv.CAP_PROP_FPS)
    if ret:
        frame_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        frame_gray_main = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        
        new_camera_matrix, roi = cv.getOptimalNewCameraMatrix(camera_matrix, distortion_coeffs, (width, height), 1, (width, height))

        # ==================== Undistort the image ====================
        undistorted_img = cv.undistort(frame, camera_matrix, distortion_coeffs, None, new_camera_matrix)
        
        # ==================== Crop the image to the valid region of interest ====================
        x, y, w, h = roi
        undistorted_img = undistorted_img[y:y+h, x:x+w]
        
        
        # ==================== Masking and filtering ====================
        #frame, result, edges = sliders_hsv.sliders(undistorted_img, mean_hsv, cubeTopArea)
        saturationThresholdMax = 190
        saturationThresholdMin = 75
        middleOfThePicture = x + w // 2
        a_forSaturationFunction = (saturationThresholdMax - saturationThresholdMin)/middleOfThePicture
        
        
        if cx == None:
            saturationThresholdMask = 190
        else:
            saturationThresholdMask = round(a_forSaturationFunction * abs(cx - middleOfThePicture) + saturationThresholdMin)
        
        #print("Current sat:", saturationThresholdMask)    
        
        lower_bound = (75, saturationThresholdMask, 170)
        upper_bound = (255, 255, 255)

        
        #Bilateral filtering to reduce noice but keep the corners and edges intact
        frame = cv.bilateralFilter(frame, 2, 50, 50)

        
        # ==================== Applying mask ====================
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)    
        mask = cv.inRange(hsv, lower_bound, upper_bound)
        result = cv.bitwise_and(frame, frame, mask=mask)
        
        gray_result = cv.cvtColor(result, cv.COLOR_BGR2GRAY)
        
        frame_thresh = cv.threshold(gray_result,1,255,cv.THRESH_OTSU)[1]
        frame_thresh = cv.medianBlur(frame_thresh, 9)
                
        edges = cv.Canny(frame_thresh, 150, 200)
        result = cv.cvtColor(result, cv.COLOR_BGR2HSV)
        
        
        contours, _ = cv.findContours(edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        
    
        if contours:
            contour = max(contours, key=cv.contourArea)
            epsilon = 0.02 * cv.arcLength(contour, True)
            approx = cv.approxPolyDP(contour, epsilon, True)
            
            hull = cv.convexHull(contour)

            # Draw the convex hull around the cube
            cv.drawContours(frame, [hull], -1, (0, 255, 0), 2)  # Draw the convex hull in green
            #cv.drawContours(frame, [contour], -1, (255, 255, 0), 2)
            
            # Calculating contour center x coordinate
            M = cv.moments(contour)
    
            # Calculate centroid x
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
            else:
                cx = 0

            
            if cv.contourArea(contour) > 200:
                mask = cv.inRange(result, (1, 1, 1), (255, 255, 255))
                mean_hsv = np.round(cv.mean(result, mask=mask)[:3])
                
                if len(approx) == 4:
                    area = cv.contourArea(contour)                    
                    
                    approx = sort_points(approx,4)
                    
                    
                    approx_stacked.append(approx)
                    

                    if len(approx_stacked) > 10:
                        approx_stacked.pop(0)
                    
                    print("Number of Points Stacked:", len(approx_stacked))
                    
                    print("===========================")
                    print(approx_stacked)
                    print("===========================")
                    
                    
                    
                    try:
                        # Calculating coords and angles
                        if coordinates != 0 :
                            previous_x = coordinates[0]
                            
                        coordinates, angles = coords(cube_points, approx, frame)
                        
                        # Calculating speed of the cube or other object
                        
                        if coordinates != 0:
                            deltaX = previous_x - coordinates[0] 
                            object_speed = round(deltaX / (1/fps),2)
                            #print("Speed:",object_speed)
                        
                    except Exception as e:
                        print(f"Coordinates error: {e}")
                elif len(approx) > 4:
                    while len(approx) > 4:
                        epsilon *= 1.1
                        approx = cv.approxPolyDP(contour, epsilon, True)
                else:
                    print("Error - incorrect number of corners")
                
                
                corner_cnt = 1
                for corner in approx:
                    x, y = corner.ravel()
                    cv.circle(frame, (x, y), 5, (0, 0, 255), -1)
                    cv.putText(frame, str(corner_cnt), (x + 10,y + 10), cv.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2, cv.LINE_AA)
                    corner_cnt += 1
                    
                        
        # Draw coordinate system
        cv.arrowedLine(frame, (10,10), (10,60), (255,255,0), 2)
        cv.putText(frame, 'Y', (15,60), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 2)
        cv.arrowedLine(frame, (10,10), (60,10), (255,0,255), 2)
        cv.putText(frame, 'X', (50,30), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,255), 2)
        
        if coordinates != 0:
            print(coordinates, angles[0])
            
            
        cv.imshow('img1',frame)
        cv.imshow('edges',edges)  
        

    else:
        print("Camera error")
        

    
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

