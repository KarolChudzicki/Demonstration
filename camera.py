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
isInside = False
isAtMiddlePoint = False
box_stacked = []

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


def coords(points, points_sorted, frame):
    points_sorted = np.array(points_sorted, dtype=np.float32)

    # Maybe use cv.SOLVEPNP_P3P, but it only works for 4 points cv.SOLVEPNP_EPNP
    if len(points_sorted) == 4:
        success, rotation_vector, translation_vector = cv.solvePnP(points, points_sorted, camera_matrix, distortion_coeffs, flags=cv.SOLVEPNP_P3P)
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

            yaw = round(np.degrees(yaw))     
            

            # # Project the 3D points to 2D image points using the pose (rvec, tvec)
            # img_points, _ = cv.projectPoints(points, rotation_vector, translation_vector, camera_matrix, distortion_coeffs)

            # # Visualize the projected points on the image (assuming 'frame' is your video frame)
            # for point in img_points:
            #     x, y = point.ravel()
            #     cv.circle(frame, (int(x), int(y)), 5, (255, 255, 0), -1)  # Draw green dots
            
            
        except Exception as e:
            print(f"Error in angle calculation: {e}")
            return None

        coordinates = np.round(coordinates_camera_space.ravel()[:3]).astype(int).tolist()
        
       
        #Remove z which has index = 2
        coordinates.pop(2)
                
        return coordinates, yaw
    else:
        print("Solve PNP error")
        return None, None

def sort_points(points, number):
    
    if number == 4:
        # Step 1: Calculate the center of the points (average of all points)
        center = np.mean(points, axis=0)
        
        # Step 2: Calculate the angle of each point relative to the center
        def calculate_angle(point, center):
            dx = point[0] - center[0]
            dy = point[1] - center[1]
            return np.arctan2(dy, dx)

        # Step 3: Sort points based on the angle
        angles = [calculate_angle(point, center) for point in points]
        sorted_indices = np.argsort(angles)
        sorted_points = points[sorted_indices]
        
        # Step 4: Return the sorted points
        return sorted_points.astype(int)

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
        
        
        h_low = cv.getTrackbarPos("H low", "Camera params")
        s_low = cv.getTrackbarPos("S low", "Camera params") 
        v_low = cv.getTrackbarPos("V low", "Camera params") 
        h_up = cv.getTrackbarPos("H upper", "Camera params") 
        s_up = cv.getTrackbarPos("S upper", "Camera params") 
        v_up = cv.getTrackbarPos("V upper", "Camera params") 
        
        dil = cv.getTrackbarPos("Dilation", "Camera params") 
        ero = cv.getTrackbarPos("Erosion", "Camera params") 
        
        
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
        
    
               
        frame_thresh = cv.adaptiveThreshold(gray_result, 255, cv.ADAPTIVE_THRESH_MEAN_C, cv.THRESH_BINARY, 13, 2)
        #frame_thresh = cv.threshold(gray_result,1,255,cv.THRESH_OTSU)[1]
        
        kernel = cv.getStructuringElement(cv.MORPH_RECT, (11, 11))
        
        # Apply Erosion
        eroded = cv.erode(frame_thresh, kernel, iterations=ero)
        
        # Apply Dilation
        dilated = cv.dilate(eroded, kernel, iterations=dil)
               
        edges = cv.Canny(dilated, 150, 200)
               
        contours, _ = cv.findContours(edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        
        
        
    
        if contours:
            contour = max(contours, key=cv.contourArea)
            
             # Calculating contour center x coordinate
            M = cv.moments(contour)
    
            # Calculate centroid x
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M['m01'] / M['m00'])
            else:
                cx = 0
                cy = 0
            
            if cx > 100 and cx < width - 100:
                isInside = True
            else:
                isInside = False
                
            if cx < 100 and cx < width - 100:
                isAtMiddlePoint = True
            else:
                isAtMiddlePoint = False
            
            rect = cv.minAreaRect(contour)  # Get the bounding rectangle
            center_rect, size_rect, angle_rect = rect
            
            center_rect = [int(center_rect[0]), int(center_rect[1])]
            #cv.circle(frame, center_rect, 5, (255, 255, 0), 2)
            
            half_width = size_rect[0] / 2
            half_height = size_rect[1] / 2

            # Define the four corners of the rectangle before rotation
            corners = np.array([
                [-half_width, -half_height],  # Top-left
                [ half_width, -half_height],  # Top-right
                [ half_width,  half_height],  # Bottom-right
                [-half_width,  half_height]   # Bottom-left
            ])
            
            # Rotation matrix
            angle_rad = np.radians(angle_rect)
            rotation_matrix = np.array([
                [np.cos(angle_rad), -np.sin(angle_rad)],
                [np.sin(angle_rad),  np.cos(angle_rad)]
            ])

            # Rotate the corners
            rotated_corners = np.dot(corners, rotation_matrix.T)

            # Translate the rotated corners back to the rectangle's center
            rotated_corners += np.array(center_rect)

            # Convert to integer coordinates
            rotated_corners = rotated_corners.astype(int)
            rotated_corners = np.array(rotated_corners)
                                               
            for point in rotated_corners:
                xr, yr = point.ravel()
                
                cv.circle(frame, (xr, yr), 5, (255, 255, 0), 2)  # Green lines

            rotated_corners_sorted = sort_points(rotated_corners,4)
            
            box_stacked.append(rotated_corners_sorted)

            if len(box_stacked) > 2:
                box_stacked.pop(0)
                mean_points = np.mean(box_stacked, axis=0).astype(int)
                mean_points_formatted = [np.array([point], dtype=np.int32) for point in mean_points]
            
            
                rotated_corners_sorted = mean_points_formatted
            
            try:
                # Calculating coords and angles
                coordinates, angles = coords(cube_points, rotated_corners_sorted, frame)
                
                # Set 0,0 point on the left side of the screen and flip x axis
                if not None:
                    coordinates[0] += width//2
                    coordinates[0] *= -1
                    
                    # Convert coordinates to meters
                    coordinates = [coordinates[0]/1000, coordinates[1]/1000]
                                         
            except Exception as e:
                print(f"Coordinates error: {e}")

            corner_cnt = 1
            for corner in rotated_corners_sorted:
                x, y = corner.ravel()
                cv.circle(frame, (x, y), 5, (0, 0, 255), -1)
                cv.putText(frame, str(corner_cnt), (x + 10,y + 10), cv.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2, cv.LINE_AA)
                corner_cnt += 1
            
            # Draw coordinate system
            cv.arrowedLine(frame, (10,10), (10,60), (255,255,0), 2)
            cv.putText(frame, 'Y', (15,60), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 2)
            cv.arrowedLine(frame, (10,10), (60,10), (255,0,255), 2)
            cv.putText(frame, 'X', (50,30), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,255), 2)
            
            # Draw detection rectangle
            cv.rectangle(frame, (100, 100), (width-100, height-100), (255,0,255), 2)
            cv.line(frame, (width//2, 100), (width//2, height-100), (255,0,255),2)
            
        
            
            
            cv.imshow('img1',frame)
            
            return coordinates, angles, isInside, isAtMiddlePoint
        else:
            cv.imshow('img1',frame)
            return None, None, None, None
        
     
    else:
        print("Camera error")
        


    