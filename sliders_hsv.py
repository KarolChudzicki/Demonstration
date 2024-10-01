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
    cv.createTrackbar('T Upper', 'HSV Selector', 0, 255, nothing)
    cv.createTrackbar('T Lower', 'HSV Selector', 0, 255, nothing)
    
    cv.createTrackbar('HL Thresh', 'HSV Selector', 0, 255, nothing)
    cv.createTrackbar('HL min length', 'HSV Selector', 0, 255, nothing)
    cv.createTrackbar('HL max gap', 'HSV Selector', 0, 255, nothing)
    
    cv.createTrackbar('Erosion', 'HSV Selector', 0, 50, nothing)
    cv.createTrackbar('Dilation', 'HSV Selector', 0, 50, nothing)
    
    cv.setTrackbarPos('H Lower', 'HSV Selector', h_l)
    cv.setTrackbarPos('S Lower', 'HSV Selector', s_l) 
    cv.setTrackbarPos('V Lower', 'HSV Selector', v_l) 
    cv.setTrackbarPos('Blur', 'HSV Selector', blur)
    cv.setTrackbarPos('T Upper', 'HSV Selector', 255)
    cv.setTrackbarPos('T Lower', 'HSV Selector', 1)
    
    cv.setTrackbarPos('HL Thresh', 'HSV Selector', 25)
    cv.setTrackbarPos('HL min length', 'HSV Selector', 15)
    cv.setTrackbarPos('HL max gap', 'HSV Selector', 50)
    


def sliders(frame):
    h_l = cv.getTrackbarPos('H Lower', 'HSV Selector')
    s_l = cv.getTrackbarPos('S Lower', 'HSV Selector')
    v_l = cv.getTrackbarPos('V Lower', 'HSV Selector')
    h_u = cv.getTrackbarPos('H Upper', 'HSV Selector')
    s_u = cv.getTrackbarPos('S Upper', 'HSV Selector')
    v_u = cv.getTrackbarPos('V Upper', 'HSV Selector')
    gauss = cv.getTrackbarPos('Blur', 'HSV Selector')
    gauss = gauss * 2 + 3
    
    
    
    t_u = cv.getTrackbarPos('T Upper','HSV Selector')
    t_l = cv.getTrackbarPos('T Lower','HSV Selector')

    
    
    hlt = cv.getTrackbarPos('HL Thresh', 'HSV Selector')
    hlm = cv.getTrackbarPos('HL min length', 'HSV Selector')
    hlg = cv.getTrackbarPos('HL max gap', 'HSV Selector')
    er = cv.getTrackbarPos('Erosion', 'HSV Selector')
    di = cv.getTrackbarPos('Dilation', 'HSV Selector')

    lower_bound = (h_l, s_l, v_l)
    upper_bound = (h_u, s_u, v_u)


    frame = cv.GaussianBlur(frame, (gauss,gauss), 0)
    frame = cv.bilateralFilter(frame, 9, 75, 75)
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    
    
    mask = cv.inRange(hsv, lower_bound, upper_bound)
    result = cv.bitwise_and(frame, frame, mask=mask)
    
    gray_result = cv.cvtColor(result, cv.COLOR_BGR2GRAY)
    
    
    
    #frame_thresh = cv.threshold(gray_result, t_l, t_u, cv.THRESH_BINARY)[1]
    frame_thresh = cv.threshold(gray_result,t_l,t_u,cv.THRESH_BINARY+cv.THRESH_OTSU)[1]
    
    #frame_thresh = cv.adaptiveThreshold(gray_result, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY, 3, 2)
    
    kernel = np.ones((3, 3), np.uint8)
    
    # Apply erosion followed by dilation (opening)
    eroded = cv.erode(frame_thresh, kernel, iterations=2)
    #opened = cv.morphologyEx(eroded, cv.MORPH_CLOSE, kernel)

    
    edges = cv.Canny(eroded, 150, 200)
    edges = cv.dilate(edges, np.ones((3, 3), np.uint8), iterations=di)
    
    
    
    lines = cv.HoughLinesP(edges, 1, np.pi / 180, hlt, minLineLength=hlm, maxLineGap=hlg)
        
    return frame, result, edges, lines


