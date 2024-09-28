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

width = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))


L = 31  # The size of the cube (known)
cube_points1 = np.array([
    [0, 0, L],  # Corner 1
    [0, 0, 0],  # Corner 2
    [0, L, 0],  # Corner 3
    [L, L, 0],  # Corner 4
    [L, 0, 0],  # Corner 5
    [L, 0, L]   # Corner 6
], dtype=np.float32)

cube_points2 = np.array([
    [0, 0, L],  # Corner 1
    [0, L, 0],  # Corner 2
    [L, L, 0],  # Corner 3
    [L, 0, L]   # Corner 4
], dtype=np.float32)


def coords(points, approx):
    image_points = np.array([point[0] for point in approx], dtype=np.float32)
    success, rotation_vector, translation_vector = cv.solvePnP(points, image_points, camera_matrix, dist_coeffs)
    rotation_matrix, _ = cv.Rodrigues(rotation_vector)
                    
    sy = math.sqrt(rotation_matrix[0,0] ** 2 + rotation_matrix[1,0] ** 2)
    
    #yaw = math.atan2(rotation_matrix[1,0],rotation_matrix[0,0])
    #pitch = math.atan2(-rotation_matrix[2,0], sy)
    #print(np.degrees(yaw))
                    
    cube_center_object_space = np.mean(points, axis=0)
    cube_center_camera_space = np.dot(rotation_matrix, cube_center_object_space.reshape(-1, 1)) + translation_vector

    # Print the 3D coordinates of the cube's center
    print("Cube Center in 3D Space (relative to camera):", cube_center_camera_space.ravel())

def sort_points(points, number):
    
    if number == 4:
        top_left = min(points, key=lambda point: point[0][0] + point[0][1])
        bottom_right = max(points, key=lambda point: point[0][0] + point[0][1])
        
        top_right = max(points, key=lambda point: point[0][0] - point[0][1])
        bottom_left = min(points, key=lambda point: point[0][0] - point[0][1])

        return [top_left, bottom_left, bottom_right, top_right]
    
    if number == 6:
        # Split points into two sets based on z-coordinate (top and bottom)
        top_points = [point for point in points if point[0][2] == np.max(points[:, 0, 2])]
        bottom_points = [point for point in points if point[0][2] == np.min(points[:, 0, 2])]
        
        # Sort top points by x, then y
        top_sorted = sorted(top_points, key=lambda point: (point[0][0], point[0][1]))
        # Sort bottom points by x, then y
        bottom_sorted = sorted(bottom_points, key=lambda point: (point[0][0], point[0][1]))

        # Combine the sorted points (top points first)
        return top_sorted + bottom_sorted
    
    else:
        print("Invalid number of points")
        return None
    

sliders_hsv.init(50,40,50,0)

while True:
    (ret, frame) = cap.read()
    if ret:
        frame_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        frame_gray_main = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        
        
        new_camera_matrix, roi = cv.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (width, height), 1, (width, height))

        # Undistort the image
        undistorted_img = cv.undistort(frame, camera_matrix, dist_coeffs, None, new_camera_matrix)
        
        # Crop the image to the valid region of interest
        x, y, w, h = roi
        undistorted_img = undistorted_img[y:y+h, x:x+w]

       
        
        #image = np.copy(frame)

        frame, result, edges, lines = sliders_hsv.sliders(undistorted_img)
        

        
        
        #lines = cv.HoughLines(edges, 1, np.pi / 180, 25)
        

        # Draw lines on the original image
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                
                dx = x2 - x1
                dy = y2 - y1
                
                x1_new = x1 - dx//2
                y1_new = y1 - dy//2
                x2_new = x2 + dx//2
                y2_new = y2 + dy//2
                                
                cv.line(frame, (x1_new, y1_new), (x2_new, y2_new), (0, 0, 255), 2)
        else:
            print("No lines detected")
        
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





'''
def estimate_corners(lines):
    if lines is not None:
        i = 0
        linesAB = np.empty((6,2))
        #Sort lines first
        if len(lines) == 6:
            lines = sort6_points(lines)
            
        for line in lines:
            x1, y1, x2, y2 = line[0]
            print(line[0][0])
            # Elongating the line
            dx = x2 - x1
            dy = y2 - y1
                
            #x1_new = x1 - dx//3
            #y1_new = y1 - dy//3
            #x2_new = x2 + dx//3
            #y2_new = y2 + dy//3
                
            # Getting line equation for the line
                
            a = dy/dx
            b = y1 - a*x1
                
            linesAB = np.vstack((linesAB, [a, b]))
                                
            #cv.line(edges, (x1, y1), (x2, y2), (255, 255, 255), 2)
            #cv.putText(edges, str(i), (x1 + 10,y1 + 10), cv.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2, cv.LINE_AA)
                        
            i += 1
        #for ab in linesAB:
            
        
        linesAB = np.empty(12)
    else:
        print("No lines detected")
'''