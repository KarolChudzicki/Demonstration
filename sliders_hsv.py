import cv2 as cv
import numpy as np

def nothing(x):
    pass


kernel = np.array([[0, -1, 0],
                    [-1, 5,-1],
                    [0, -1, 0]])

# Create a window
def init(h_l, s_l, v_l, blur):
    cv.namedWindow('HSV Selector')
    cv.resizeWindow('HSV Selector', 800, 600)

    # Create trackbars for HSV values
    cv.createTrackbar('H Lower', 'HSV Selector', 0, 179, nothing)
    cv.createTrackbar('S Lower', 'HSV Selector', 0, 255, nothing)
    cv.createTrackbar('V Lower', 'HSV Selector', 0, 255, nothing)
    cv.createTrackbar('H Upper', 'HSV Selector', 179, 179, nothing)
    cv.createTrackbar('S Upper', 'HSV Selector', 255, 255, nothing)
    cv.createTrackbar('V Upper', 'HSV Selector', 255, 255, nothing)
    cv.createTrackbar('Blur', 'HSV Selector', 0, 15, nothing)
    cv.createTrackbar('Sigma X', 'HSV Selector', 0, 100, nothing)
    cv.createTrackbar('T Upper', 'HSV Selector', 0, 255, nothing)
    cv.createTrackbar('T Lower', 'HSV Selector', 0, 255, nothing)
    cv.createTrackbar('Kernel for opening', 'HSV Selector', 0, 15, nothing)
    
    cv.setTrackbarPos('H Lower', 'HSV Selector', h_l)
    cv.setTrackbarPos('S Lower', 'HSV Selector', s_l) 
    cv.setTrackbarPos('V Lower', 'HSV Selector', v_l) 
    cv.setTrackbarPos('Blur', 'HSV Selector', blur)
    cv.setTrackbarPos('T Upper', 'HSV Selector', 255)
    cv.setTrackbarPos('T Lower', 'HSV Selector', 1)
    cv.setTrackbarPos('Kernel for opening', 'HSV Selector', 2)
    


def sliders(frame):
    h_l = cv.getTrackbarPos('H Lower', 'HSV Selector')
    s_l = cv.getTrackbarPos('S Lower', 'HSV Selector')
    v_l = cv.getTrackbarPos('V Lower', 'HSV Selector')
    h_u = cv.getTrackbarPos('H Upper', 'HSV Selector')
    s_u = cv.getTrackbarPos('S Upper', 'HSV Selector')
    v_u = cv.getTrackbarPos('V Upper', 'HSV Selector')
    gauss = cv.getTrackbarPos('Blur', 'HSV Selector')
    gauss = gauss * 2 + 3
    sigmaX = cv.getTrackbarPos('Sigma X','HSV Selector')
    sigmaX = sigmaX / 100 + 0.1
    frame = cv.GaussianBlur(frame, (gauss,gauss), 0)
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    
    
    t_u = cv.getTrackbarPos('T Upper','HSV Selector')
    t_l = cv.getTrackbarPos('T Lower','HSV Selector')
    opening = cv.getTrackbarPos('Kernel for opening', 'HSV Selector')
    opening = opening * 2 + 3

    lower_bound = (h_l, s_l, v_l)
    upper_bound = (h_u, s_u, v_u)

    #hsv = cv.filter2D(hsv, -1, kernel)
    #kernel = np.ones((gauss, gauss), np.uint8)
    #hsv = cv.dilate(hsv, kernel, iterations=1)
    
    mask = cv.inRange(hsv, lower_bound, upper_bound)
    result = cv.bitwise_and(frame, frame, mask=mask)
    
    gray_result = cv.cvtColor(result, cv.COLOR_BGR2GRAY)
    
    
    frame_thresh = cv.threshold(gray_result, t_l, t_u, cv.THRESH_BINARY)[1]
    
    kernel = np.ones((opening, opening), np.uint8)
    
    # Apply erosion followed by dilation (opening)
    eroded = cv.erode(frame_thresh, kernel, iterations=1)
    opened = cv.dilate(eroded, kernel, iterations=1)
    
    # Sharpening kernel
    kernel_sharpen = np.array([[-1, -1, -1],
                               [-1,  9,- 1],
                               [-1, -1, -1]])
    # Applying the sharpening filter
    opened = cv.filter2D(opened, -1, kernel_sharpen)
        
    return frame, result, opened


