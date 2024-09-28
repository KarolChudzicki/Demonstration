import cv2 as cv #opencv-python
import time
import numpy as np
import imutils #imutils
import math
import sliders_hsv


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

angle = 0
sum_loop = 0

width = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))

xyz = np.empty((0,3))

L = 30  # The size of the cube (known)
P = L*math.sqrt(2)/2
cube_points1 = np.array([
    [0, 0, P*2],  # Corner 1
    [-P, 0, P],  # Corner 2
    [-P, L+2, P],  # Corner 3
    [0, L+2, 0],  # Corner 4
    [P, L+2, P],   # Corner 5
    [P, 0, P]  # Corner 6
], dtype=np.float32)

cube_points2 = np.array([
    [-L/2, 0, L],  # Corner 1
    [-L/2, L+2, 0],  # Corner 2
    [L/2, L+2, 0],  # Corner 3
    [L/2, 0, L]   # Corner 4
], dtype=np.float32)


def coords(points, approx):
    image_points = np.array([point[0] for point in approx], dtype=np.float32)
    # Maybe use cv.SOLVEPNP_P3P, but it only works for 4 points cv.SOLVEPNP_EPNP
    if len(approx) == 4:
        success, rotation_vector, translation_vector = cv.solvePnP(points, image_points, camera_matrix, distortion_coeffs, flags=cv.SOLVEPNP_P3P)
    else:
        success, rotation_vector, translation_vector = cv.solvePnP(points, image_points, camera_matrix, distortion_coeffs, flags=cv.SOLVEPNP_EPNP)
    
    if success:
        rotation_matrix, _ = cv.Rodrigues(rotation_vector)
        cube_center_object_space = np.mean(points, axis=0)
        cube_center_camera_space = np.dot(rotation_matrix, cube_center_object_space.reshape(-1, 1)) + translation_vector

        # Calculate actual coords in real world
        rotation_matrix_inv = np.linalg.inv(rotation_matrix)
        cube_center_world_space = np.dot(rotation_matrix_inv, cube_center_camera_space)
        
            
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
            
            
            if len(approx) == 6:
                yaw += 45

            # Project the 3D points to 2D image points using the pose (rvec, tvec)
            img_points, _ = cv.projectPoints(points, rotation_vector, translation_vector, camera_matrix, distortion_coeffs)

            # Visualize the projected points on the image (assuming 'frame' is your video frame)
            for point in img_points:
                x, y = point.ravel()
                cv.circle(frame, (int(x), int(y)), 5, (255, 255, 0), -1)  # Draw green dots
            
            
        except Exception as e:
            print(f"Error in angle calculation: {e}")
            return None

        coordinates = np.round(cube_center_world_space.ravel()).astype(int)
        coordinates = [coordinates[0], coordinates[1], coordinates[2]]
        angles = [yaw, pitch, roll]
        
        return coordinates, angles
    else:
        print("Solve PNP error")

def sort_points(points, number):
    
    if number == 4 or number == 5:
        top_left = min(points, key=lambda point: point[0][0] + point[0][1])
        bottom_right = max(points, key=lambda point: point[0][0] + point[0][1])
        
        top_right = max(points, key=lambda point: point[0][0] - point[0][1])
        bottom_left = min(points, key=lambda point: point[0][0] - point[0][1])

        return [top_left, bottom_left, bottom_right, top_right]
    
    #if number == 5:
        
    
        
    
    if number == 6:
        top = min(points, key=lambda point: point[0][1])
        bottom = max(points, key=lambda point: point[0][1])
            
        sort_by_x = sorted(points, key=lambda point: point[0][0])
        left_points = sort_by_x[:2]
        right_points = sort_by_x[4:]
            
        left_points_sorted_by_y = sorted(left_points, key=lambda point: point[0][1])
        right_points_sorted_by_y = sorted(right_points, key=lambda point: point[0][1], reverse=True)
            
            
        
        return [top, left_points_sorted_by_y[0], left_points_sorted_by_y[1], bottom, right_points_sorted_by_y[0], right_points_sorted_by_y[1]]
    
    else:
        print("Invalid number of points")
        return None


    
# Initialise the sliders
sliders_hsv.init(50,40,50,2)


#================================ MAIN LOOP ================================
while True:
    (ret, frame) = cap.read()
    if ret:
        frame_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        frame_gray_main = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        
        
        new_camera_matrix, roi = cv.getOptimalNewCameraMatrix(camera_matrix, distortion_coeffs, (width, height), 1, (width, height))

        # Undistort the image
        undistorted_img = cv.undistort(frame, camera_matrix, distortion_coeffs, None, new_camera_matrix)

        
        # Crop the image to the valid region of interest
        x, y, w, h = roi
        undistorted_img = undistorted_img[y:y+h, x:x+w]

        frame, result, edges, lines = sliders_hsv.sliders(undistorted_img)
        contours, _ = cv.findContours(edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        
    
        if contours:
            contour = max(contours, key=cv.contourArea)
            epsilon = 0.02 * cv.arcLength(contour, True)
            approx = cv.approxPolyDP(contour, epsilon, True)
            
            hull = cv.convexHull(contour)

            # Draw the convex hull around the cube
            cv.drawContours(frame, [hull], -1, (0, 255, 0), 2)  # Draw the convex hull in green

            if cv.contourArea(contour) > 200 and len(approx) < 7:
                if len(approx) == 6:
                    approx = sort_points(approx,6)
                    coordinates, angles = coords(cube_points1, approx)
                
                elif len(approx) == 4:
                    approx = sort_points(approx,4)
                    coordinates, angles = coords(cube_points2, approx)
                elif len(approx) == 5:
                    approx = sort_points(approx,5)
                    coordinates, angles = coords(cube_points2, approx)
                else:
                    print("Error - incorrect number of corners")
                
                
                corner_cnt = 1
                for corner in approx:
                    x, y = corner.ravel()
                    cv.circle(frame, (x, y), 5, (0, 0, 255), -1)
                    cv.putText(frame, str(corner_cnt), (x + 10,y + 10), cv.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2, cv.LINE_AA)
                            
                    corner_cnt += 1
                    
                # Coordinates processing (removing outliers)
                
                if coordinates != None:
                    xyz = np.vstack([xyz,coordinates])
                    if xyz.shape[0] > 20:
                        xyz = np.delete(xyz, 0, axis = 0)
                        mean = np.mean(xyz, axis = 0)
                        std_dev = np.std(xyz, axis = 0)
                        score = np.abs((xyz-mean)/std_dev)
                            
                        xyz = xyz[(score > 0).all(axis=1)] 
                        if xyz.shape[0] > 1:
                            print("Cube Center (relative to the camera plane):",xyz[-1], " Yaw, pitch and roll of the cube:", 0)
                        
                #print(xyz)
                
                    
        
                           
        # Draw coordinate system
        cv.arrowedLine(frame, (10,10), (10,60), (255,255,0), 2)
        cv.putText(frame, 'Y', (15,60), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 2)
        cv.arrowedLine(frame, (10,10), (60,10), (255,0,255), 2)
        cv.putText(frame, 'X', (50,30), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,255), 2)
        
        
        
        cv.imshow('img1',frame)
        cv.imshow('esges',edges)
    else:
        print("Camera error")
        break
    
    if cv.waitKey(1) & 0xFF == ord('q'):
        break




