import numpy as np
import cv2 as cv
import camera
from camera import *


camera_coords = []
arm_coords = []
coords_saved = 0

    

while True:
    camera_frame_coords, camera_frame_angles, frame, edges = camera.run()
    
    if cv.waitKey(1) & 0xFF == ord('p'):
        if camera_frame_coords == -1:
            print("Invalid coordinates")
        else:
            print("Coords saved!")
            camera_coords.append(camera_frame_coords)
            coords_saved += 1
        
    if coords_saved > 5:
        break
    
print("================================================================================")    
print(camera_coords)
print("================================================================================")
print(arm_coords)
print("================================================================================")