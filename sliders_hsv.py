import cv2 as cv
import numpy as np

def nothing(x):
    pass






# Create a window
def init(h_l, s_l, v_l, s_u, v_u, bilD, bilC, bilS):
    cv.namedWindow('HSV Selector')
    cv.resizeWindow('HSV Selector', 800, 700)

    # Create trackbars for HSV values
    cv.createTrackbar('H Lower', 'HSV Selector', 0, 179, nothing)
    cv.createTrackbar('S Lower', 'HSV Selector', 0, 255, nothing)
    cv.createTrackbar('V Lower', 'HSV Selector', 0, 255, nothing)
    cv.createTrackbar('H Upper', 'HSV Selector', 179, 179, nothing)
    cv.createTrackbar('S Upper', 'HSV Selector', 255, 255, nothing)
    cv.createTrackbar('V Upper', 'HSV Selector', 255, 255, nothing)
    cv.createTrackbar('T Upper', 'HSV Selector', 0, 255, nothing)
    cv.createTrackbar('T Lower', 'HSV Selector', 0, 255, nothing)
    
    cv.createTrackbar('HL Thresh', 'HSV Selector', 0, 255, nothing)
    cv.createTrackbar('HL min length', 'HSV Selector', 0, 255, nothing)
    cv.createTrackbar('HL max gap', 'HSV Selector', 0, 255, nothing)
    
    cv.createTrackbar('Erosion', 'HSV Selector', 0, 50, nothing)
    cv.createTrackbar('Dilation', 'HSV Selector', 0, 50, nothing)
    
    cv.createTrackbar('Bil D', 'HSV Selector', 1, 20, nothing)
    cv.createTrackbar('Bil Color', 'HSV Selector', 1, 200, nothing)
    cv.createTrackbar('Bil Space', 'HSV Selector', 1, 200, nothing)
    
    cv.setTrackbarPos('H Lower', 'HSV Selector', h_l)
    cv.setTrackbarPos('S Lower', 'HSV Selector', s_l) 
    cv.setTrackbarPos('V Lower', 'HSV Selector', v_l)
    cv.setTrackbarPos('S Lower', 'HSV Selector', s_u) 
    cv.setTrackbarPos('V Lower', 'HSV Selector', v_u)  
    cv.setTrackbarPos('T Upper', 'HSV Selector', 255)
    cv.setTrackbarPos('T Lower', 'HSV Selector', 1)
    
    cv.setTrackbarPos('HL Thresh', 'HSV Selector', 25)
    cv.setTrackbarPos('HL min length', 'HSV Selector', 15)
    cv.setTrackbarPos('HL max gap', 'HSV Selector', 50)
    
    cv.setTrackbarPos('Bil D', 'HSV Selector', bilD)
    cv.setTrackbarPos('Bil Color', 'HSV Selector', bilC)
    cv.setTrackbarPos('Bil Space', 'HSV Selector', bilS)
    


def sliders(frame, mean_hsv):
    h_l = cv.getTrackbarPos('H Lower', 'HSV Selector')
    s_l = cv.getTrackbarPos('S Lower', 'HSV Selector')
    v_l = cv.getTrackbarPos('V Lower', 'HSV Selector')
    h_u = cv.getTrackbarPos('H Upper', 'HSV Selector')
    s_u = cv.getTrackbarPos('S Upper', 'HSV Selector')
    v_u = cv.getTrackbarPos('V Upper', 'HSV Selector')
    
    t_u = cv.getTrackbarPos('T Upper','HSV Selector')
    t_l = cv.getTrackbarPos('T Lower','HSV Selector')

    hlt = cv.getTrackbarPos('HL Thresh', 'HSV Selector')
    hlm = cv.getTrackbarPos('HL min length', 'HSV Selector')
    hlg = cv.getTrackbarPos('HL max gap', 'HSV Selector')
    er = cv.getTrackbarPos('Erosion', 'HSV Selector')
    di = cv.getTrackbarPos('Dilation', 'HSV Selector')
    
    


    lower_bound = (h_l, s_l, v_l)
    upper_bound = (h_u, s_u, v_u)

    bilD = cv.getTrackbarPos('Bil D', 'HSV Selector')
    bilC = cv.getTrackbarPos('Bil Color', 'HSV Selector')
    bilS = cv.getTrackbarPos('Bil Space', 'HSV Selector')

    
    #Bilateral filtering to reduce noice but keep the corners and edges intact
    frame = cv.bilateralFilter(frame, bilD, bilC, bilS)

    
    # Applying mask
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)    
    mask = cv.inRange(hsv, lower_bound, upper_bound)
    result = cv.bitwise_and(frame, frame, mask=mask)
    
    gray_result = cv.cvtColor(result, cv.COLOR_BGR2GRAY)
    

    
    # Thresholding the image
    #frame_thresh = cv.threshold(gray_result,t_l,t_u,cv.THRESH_OTSU)[1]
    
    frame_thresh = cv.adaptiveThreshold(gray_result, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY, 15, 4)
    frame_thresh = cv.medianBlur(frame_thresh, 3)
    
    
    kernel = np.ones((3, 3), np.uint8)
    
    # Apply erosion followed by dilation (opening)
    sharpening_kernel = np.array([[0, -1, 0], 
                              [-1, 5, -1], 
                              [0, -1, 0]])
    #sharpened_image = cv.filter2D(frame_thresh, -1, sharpening_kernel)
    
    #opened = cv.morphologyEx(frame_thresh, cv.MORPH_OPEN, kernel)
    
    #closed = cv.morphologyEx(sharpened_image, cv.MORPH_CLOSE, kernel)
    
    
    edges = cv.Canny(frame_thresh, 150, 200)
    edges = cv.dilate(edges, kernel, iterations=di)
    edges = cv.erode(edges, kernel, iterations=er)
    
    
    
    
    result = cv.cvtColor(result, cv.COLOR_BGR2HSV)
    
        
    return frame, result, edges


