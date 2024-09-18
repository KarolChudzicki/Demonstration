import cv2 as cv

def nothing(x):
    pass

# Create a window
def init(h_l, s_l, v_l, blur):
    cv.namedWindow('HSV Selector')

    # Create trackbars for HSV values
    cv.createTrackbar('H Lower', 'HSV Selector', 0, 179, nothing)
    cv.createTrackbar('S Lower', 'HSV Selector', 0, 255, nothing)
    cv.createTrackbar('V Lower', 'HSV Selector', 0, 255, nothing)
    cv.createTrackbar('H Upper', 'HSV Selector', 179, 179, nothing)
    cv.createTrackbar('S Upper', 'HSV Selector', 255, 255, nothing)
    cv.createTrackbar('V Upper', 'HSV Selector', 255, 255, nothing)
    cv.createTrackbar('Blur', 'HSV Selector', 0, 15, nothing)
    
    cv.setTrackbarPos('H Lower', 'HSV Selector', h_l)
    cv.setTrackbarPos('S Lower', 'HSV Selector', s_l) 
    cv.setTrackbarPos('V Lower', 'HSV Selector', v_l) 
    cv.setTrackbarPos('Blur', 'HSV Selector', blur)
    


def sliders(frame):
    h_l = cv.getTrackbarPos('H Lower', 'HSV Selector')
    s_l = cv.getTrackbarPos('S Lower', 'HSV Selector')
    v_l = cv.getTrackbarPos('V Lower', 'HSV Selector')
    h_u = cv.getTrackbarPos('H Upper', 'HSV Selector')
    s_u = cv.getTrackbarPos('S Upper', 'HSV Selector')
    v_u = cv.getTrackbarPos('V Upper', 'HSV Selector')
    gauss = cv.getTrackbarPos('Blur', 'HSV Selector')
    gauss = gauss * 2 + 3
    frame = cv.GaussianBlur(frame, (gauss,gauss), 0)
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    lower_bound = (h_l, s_l, v_l)
    upper_bound = (h_u, s_u, v_u)


    mask = cv.inRange(hsv, lower_bound, upper_bound)
    result = cv.bitwise_and(frame, frame, mask=mask)
    
    _, frame_thresh = cv.threshold(result, 0, 255, cv.THRESH_BINARY)
        
        
    return frame, result, frame_thresh


