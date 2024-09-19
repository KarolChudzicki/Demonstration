import cv2 as cv
import numpy as np
import glob




def run():
    #======================= CHESSBOARD PARAMETERS =======================#
    chessboard_size = (8, 12)
    square_size = 19
    frame_size = (1920, 1080)

    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    #======================= CHESSBOARD COORDINATES =======================#
    chessboard_coords = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)

    # Set the X and Y coordinates of the object points (Z remains 0)
    chessboard_coords[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)

    obj_points = []  # 3D points in real world space
    img_points = []  # 2D points in image plane

    calib_images = glob.glob('calibration_images/captured_image*.jpg')

    for image in calib_images:
        img = cv.imread(image)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        
        ret, corners = cv.findChessboardCorners(gray, chessboard_size, None)
        
        if ret:
            # Add object points (3D) and image points (2D) after detecting corners
            obj_points.append(chessboard_coords)
            img_points.append(corners)
            
            # Draw and display the corners on the chessboard image
            cv.drawChessboardCorners(img, chessboard_size, corners, ret)
            #cv.imshow('Chessboard', img)
            cv.waitKey(100)
        else:
            print(f"Chessboard corners not detected in {image}")
            
    cv.destroyAllWindows()


    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv.calibrateCamera(obj_points, img_points, gray.shape[::-1], None, None)
    print(camera_matrix, dist_coeffs, rvecs, tvecs)
    
    if ret:
        return camera_matrix, dist_coeffs, rvecs, tvecs
    else:
        return None
