import cv2 as cv #opencv-python
import time
import numpy as np
import imutils #imutils
import math



camera_matrix = np.array([
    [688.32981535, 0.0, 399.73760407],
    [0.0, 693.39476642, 229.01366837],
    [0.0, 0.0, 1.0]
])

# Distance Coefficients
distortion_coeffs = np.array([
    [-0.46325548, 0.39960969, -0.00700461, -0.00760831, -0.3348135]
])

# Rotation Vectors
rotation_vectors = np.array([
    [[-0.7403499], [0.58274953], [1.24738629]],
    [[-0.57127351], [-0.77564103], [-1.35487777]],
    [[-0.60937652], [-0.69686486], [-1.17755222]],
    [[-0.63056647], [-0.69631842], [-1.02640127]],
    [[1.01089936], [0.95517173], [-1.22004167]],
    [[-0.6645012], [-0.95390788], [-1.4396553]],
    [[-0.48320119], [-0.74555694], [-1.5094723]],
    [[-0.37069059], [-0.63566572], [-1.55918669]],
    [[-0.85021161], [0.4132406], [1.12427004]],
    [[-0.60855108], [-0.65922994], [-1.14164917]],
    [[-0.74883804], [-0.88594991], [-1.42476713]],
    [[-0.43360531], [-0.63729615], [-1.50290743]],
    [[-0.67681829], [-1.12381021], [-1.25310231]],
    [[-0.68836603], [-0.92376294], [-1.41817524]],
    [[-0.60385296], [-0.80618265], [-1.46290843]],
    [[-0.51818054], [-0.70023353], [-1.49262764]],
    [[-0.44931213], [-0.62174755], [-1.5106191]],
    [[-0.95279976], [0.52381375], [1.08849958]],
    [[-0.51187086], [-0.93088351], [-1.52075706]]
])

# Translation Vectors
translation_vectors = np.array([
    [[2.33409013e+00], [-5.58628087e+00], [1.99650270e+01]],
    [[-6.83665653e+00], [-1.52314455e-01], [1.46692913e+01]],
    [[-7.12309045e+00], [-1.18419889e+00], [1.46135454e+01]],
    [[-7.27601442e+00], [-1.95801311e+00], [1.40673987e+01]],
    [[-8.06637865e+00], [-2.28339564e+00], [2.04809791e+01]],
    [[-7.32640419e+00], [1.04260216e+00], [1.47601667e+01]],
    [[-7.31753324e+00], [5.18592165e-01], [1.47188155e+01]],
    [[-7.39184761e+00], [-5.38357551e-03], [1.48079788e+01]],
    [[1.24604433e+00], [-5.51257469e+00], [2.07103458e+01]],
    [[-7.72811598e+00], [-1.48184886e+00], [1.48004974e+01]],
    [[-7.30460803e+00], [-4.65293770e-01], [1.41760988e+01]],
    [[-7.45818562e+00], [-2.22963449e-01], [1.58474980e+01]],
    [[-7.02422412e+00], [-2.62069251e-01], [1.28563734e+01]],
    [[-7.16625983e+00], [1.74076365e+00], [1.33049735e+01]],
    [[-7.15461970e+00], [1.62740250e+00], [1.31957644e+01]],
    [[-7.13769778e+00], [1.43070279e+00], [1.30029796e+01]],
    [[-7.15790953e+00], [1.25185122e+00], [1.27403358e+01]],
    [[1.37661952e+00], [-4.42426098e+00], [2.15018129e+01]],
    [[-6.44849354e+00], [6.92560932e-01], [1.38601208e+01]]
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
is_cx_calculated = False
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

def initSlider():
    cv.namedWindow("Camera params",cv.WINDOW_NORMAL)
    cv.createTrackbar("H low", "Camera params", 75, 255, lambda x: None)
    cv.createTrackbar("S low", "Camera params", 150, 255, lambda x: None)
    cv.createTrackbar("V low", "Camera params", 170, 255, lambda x: None)
    
    cv.createTrackbar("H upper", "Camera params", 255, 255, lambda x: None)
    cv.createTrackbar("S upper", "Camera params", 255, 255, lambda x: None)
    cv.createTrackbar("V upper", "Camera params", 255, 255, lambda x: None)
    
    cv.createTrackbar("Dilation", "Camera params", 6, 10, lambda x: None)
    cv.createTrackbar("Erosion", "Camera params", 5, 10, lambda x: None)
    
    cv.createTrackbar("Block size", "Camera params", 8, 20, lambda x: None)
    cv.createTrackbar("K size", "Camera params", 8, 15, lambda x: None)
    cv.createTrackbar("K", "Camera params", 105, 200, lambda x: None)
    
    cv.createTrackbar("Min dist", "Camera params", 10, 500, lambda x: None)
    cv.createTrackbar("Quality", "Camera params", 10, 500, lambda x: None)
        
def update():
    pass


#================================ MAIN LOOP ================================
def run():
    global is_cx_calculated, coordinates, angles, cx
    
    (ret, frame) = cap.read()
    fps = cap.get(cv.CAP_PROP_FPS)
    if ret:
       
        new_camera_matrix, roi = cv.getOptimalNewCameraMatrix(camera_matrix, distortion_coeffs, (width, height), 1, (width, height))

        # ==================== Undistort the image ====================
        undistorted_img = cv.undistort(frame, camera_matrix, distortion_coeffs, None, new_camera_matrix)
        
        # ==================== Crop the image to the valid region of interest ====================
        x, y, w, h = roi
        undistorted_img = undistorted_img[y:y+h, x:x+w]
        
        
        # ==================== Masking and filtering ====================
        #frame, result, edges = sliders_hsv.sliders(undistorted_img, mean_hsv, cubeTopArea)
        # saturationThresholdMax = 190
        # saturationThresholdMin = 75
        # middleOfThePicture = x + w // 2
        # a_forSaturationFunction = (saturationThresholdMax - saturationThresholdMin)/middleOfThePicture
        
        
        # if not is_cx_calculated:
        #     saturationThresholdMask = 190
        # else:
        #     try: 
        #         saturationThresholdMask = round(a_forSaturationFunction * abs(cx - middleOfThePicture) + saturationThresholdMin)
        #     except Exception as e:
        #         print(f"CX error: {e}")
        #         saturationThresholdMask = 190
        
        h_low = cv.getTrackbarPos("H low", "Camera params")
        s_low = cv.getTrackbarPos("S low", "Camera params") 
        v_low = cv.getTrackbarPos("V low", "Camera params") 
        h_up = cv.getTrackbarPos("H upper", "Camera params") 
        s_up = cv.getTrackbarPos("S upper", "Camera params") 
        v_up = cv.getTrackbarPos("V upper", "Camera params") 
        
        dil = cv.getTrackbarPos("Dilation", "Camera params") 
        ero = cv.getTrackbarPos("Erosion", "Camera params") 
        
        b_size = cv.getTrackbarPos("Block size", "Camera params") 
        k_size = cv.getTrackbarPos("K size", "Camera params") *2 +1
        k_harris = cv.getTrackbarPos("K", "Camera params")/1000
        
        minDistance = cv.getTrackbarPos("Min dist", "Camera params")
        quality = cv.getTrackbarPos("Quality", "Camera params") / 1000
        
        lower_bound = (h_low, s_low, v_low)
        upper_bound = (h_up, s_up, v_up)

        
        #Bilateral filtering to reduce noice but keep the corners and edges intact
        #frame = cv.bilateralFilter(frame, 7, 50, 50)

        
        # ==================== Applying mask ====================
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)    
        mask = cv.inRange(hsv, lower_bound, upper_bound)
        
        # kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
        # mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
        frame = cv.medianBlur(frame, 3)
        
        result = cv.bitwise_and(hsv, frame, mask=mask)
        
        gray_result = cv.cvtColor(result, cv.COLOR_BGR2GRAY)
        
        result_harris = result.copy()
        result_harris = cv.cvtColor(result_harris, cv.COLOR_BGR2GRAY)
        cv.imshow('result',result_harris) 
        
        # harris_test = np.float32(harris_test)
        dst = cv.cornerHarris(result_harris, b_size, k_size, k_harris)
        
        dst = cv.dilate(dst, None)
        result_harris_bgr = cv.cvtColor(result_harris, cv.COLOR_GRAY2BGR)
        result_harris_bgr[dst > 0.01 * dst.max()] = [0, 0, 255]
        
        if dst.any() > 0:
            best_corners = cv.goodFeaturesToTrack(dst, maxCorners=4, qualityLevel=quality, minDistance=minDistance)
        else:
            best_corners = []
        print(best_corners)
                
        cv.imshow('harris',result_harris_bgr) 
        
        
        if len(best_corners) > 0:
            sorted_corners = sort_points(best_corners,4)
            approx_stacked.append(sorted_corners)

            if len(approx_stacked) > 3:
                approx_stacked.pop(0)
                        

                        
            mean_points = np.mean(approx_stacked, axis=0).astype(int)
            mean_points_formatted = [np.array([point], dtype=np.int32) for point in mean_points]
                        
            
                        
            try:
                coordinates, angles = coords(cube_points, sorted_corners, frame)
                
            except Exception as e:
                print(f"Coordinates error: {e}")
            
            corner_cnt = 1
            for corner in sorted_corners:
                x, y = corner.ravel()
                x = int(x)
                y = int(y)
                cv.circle(frame, (x, y), 5, (0, 0, 255), 2)
                cv.putText(frame, str(corner_cnt), (x + 10,y + 10), cv.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2, cv.LINE_AA)
                corner_cnt += 1
        
        # frame_thresh = cv.adaptiveThreshold(gray_result, 255, cv.ADAPTIVE_THRESH_MEAN_C, cv.THRESH_BINARY, 13, 2)
        # #frame_thresh = cv.threshold(gray_result,1,255,cv.THRESH_OTSU)[1]
        
        # kernel = cv.getStructuringElement(cv.MORPH_RECT, (11, 11))
        
        # # Apply Erosion
        # eroded = cv.erode(frame_thresh, kernel, iterations=ero)
        
        # # Apply Dilation
        # dilated = cv.dilate(eroded, kernel, iterations=dil)

        
                
        # edges = cv.Canny(dilated, 150, 200)
        
        
        
        # result = cv.cvtColor(result, cv.COLOR_BGR2HSV)
        
        
        # contours, _ = cv.findContours(edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        
        
    
        # if contours:
        #     contour = max(contours, key=cv.contourArea)
        #     epsilon = 0.001 * cv.arcLength(contour, True)
        #     approx = cv.approxPolyDP(contour, epsilon, True)
            
        #     hull = cv.convexHull(contour)

        #     # Draw the convex hull around the cube
        #     cv.drawContours(frame, [hull], -1, (0, 255, 0), 2)  # Draw the convex hull in green
            
        #     # Calculating contour center x coordinate
        #     M = cv.moments(contour)
    
        #     # Calculate centroid x
        #     if M["m00"] != 0:
        #         cx = int(M["m10"] / M["m00"])
        #     else:
        #         cx = 0
                
        #     is_cx_calculated = True

            
        #     if cv.contourArea(contour) > 100:
        #         mask = cv.inRange(result, (1, 1, 1), (255, 255, 255))
        #         # mean_hsv = np.round(cv.mean(result, mask=mask)[:3])
        #         #print("len approx", len(approx))
        #         if len(approx) == 4:
        #             # area = cv.contourArea(contour)                    
        #             print("Contour")
        #             approx = sort_points(approx,4)
        #             approx_stacked.append(approx)

        #             if len(approx_stacked) > 3:
        #                 approx_stacked.pop(0)
                    
        #             # print("Number of Points Stacked:", len(approx_stacked))
        #             # print("===========================")
        #             # print(approx_stacked)
        #             # print("===========================")
                    
        #             mean_points = np.mean(approx_stacked, axis=0).astype(int)
        #             mean_points_formatted = [np.array([point], dtype=np.int32) for point in mean_points]
                    
        #             approx = mean_points_formatted
        #             #approx = np.mean(approx_stacked, axis=0)
                    
        #             try:
        #                 # Calculating coords and angles
        #                 if coordinates != 0 :
        #                     previous_x = coordinates[0]
                            
        #                 coordinates, angles = coords(cube_points, approx, frame)
                        
        #                 # Calculating speed of the cube or other object
                        
        #                 if coordinates != 0:
        #                     deltaX = previous_x - coordinates[0] 
        #                     object_speed = round(deltaX / (1/fps),2)
        #                     #print("Speed:",object_speed)
                        
        #             except Exception as e:
        #                 print(f"Coordinates error: {e}")
        #         elif len(approx) > 4:
        #             while len(approx) > 4:
        #                 epsilon *= 1.1
        #                 approx = cv.approxPolyDP(contour, epsilon, True)
        #         else:
        #             print("Error - incorrect number of corners")
                
                
        #         corner_cnt = 1
        #         for corner in approx:
        #             x, y = corner.ravel()
        #             cv.circle(frame, (x, y), 5, (0, 0, 255), -1)
        #             cv.putText(frame, str(corner_cnt), (x + 10,y + 10), cv.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2, cv.LINE_AA)
        #             corner_cnt += 1
                    
                        
        # Draw coordinate system
        cv.arrowedLine(frame, (10,10), (10,60), (255,255,0), 2)
        cv.putText(frame, 'Y', (15,60), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 2)
        cv.arrowedLine(frame, (10,10), (60,10), (255,0,255), 2)
        cv.putText(frame, 'X', (50,30), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,255), 2)
        
        cv.imshow('img1',frame)
        #cv.imshow('edges',edges)  
        
        print(coordinates, angles)
        edges = 0
        
        if coordinates != 0:
            return coordinates, angles[0], frame, edges
        else:
            return -1, -1, frame, edges
            
            
        
        

    else:
        print("Camera error")
        


    