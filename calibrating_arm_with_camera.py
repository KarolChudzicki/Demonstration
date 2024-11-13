import numpy as np
import cv2 as cv
import camera
import ur

number_of_pictures = 10

camera_coords = []
arm_coords = []

while number_of_pictures > 0:
    camera_frame_coords = camera.run()
    arm_frame_coords = ur.currentPos()
    if camera_frame_coords == -1:
        print("Invalid coordinates")
    else:
        camera_coords.append(camera_frame_coords)
        arm_coords.append(arm_frame_coords)
        number_of_pictures -= 1
    
print("====================")    
print(camera_coords)
print("====================")
print(arm_coords)
print("====================")