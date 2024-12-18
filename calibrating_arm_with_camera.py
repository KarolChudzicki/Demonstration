import numpy as np
import cv2 as cv
import camera
from camera import *
import tkinter as tk
import gripper
import ur

camera_points = []
arm_points = []
coords_saved = 0

stop = False
coords_taken = 3

# Create main window
root = tk.Tk()
root.title("Camera-Robot Calibration")
root.geometry("400x400")  # Adjust size as needed

gripper = gripper.Gripper()
URRobot = ur.URRobot()
print(URRobot.current_Position())

gripper.connect()
#gripper.activate()

camera.initSlider()
    
        
def end_calib():
    global stop
    cv.destroyAllWindows()
    root.destroy()
    stop = True

def button_open_gripper():
    gripper.open_close(POSITION_REQUEST=85, SPEED=10, FORCE=1)
    
def button_close_gripper():
    gripper.open_close(POSITION_REQUEST=0, SPEED=10, FORCE=1)
    
def save_coords():
    global coords_saved
    if camera_frame_coords == None:
            print("Invalid coordinates")
    else:
        camera_points.append(camera_frame_coords)
        # Get only X and Y position of the arm
        arm_xy = URRobot.current_Position()[:2].tolist()
        arm_points.append(arm_xy)
        coords_saved += 1
        print("Coords saved!", arm_points)
    
    
def activate_gripper():
    gripper.activate()

# Add buttons
stop_button = tk.Button(root, text="Stop Calibration", width=20, height=2, command = end_calib)
stop_button.pack(pady=10)

act_button = tk.Button(root, text="Activate gripper", width=20, height=2, command = activate_gripper)
act_button.pack(pady=10)

open_gripper = tk.Button(root, text="Open gripper", width=20, height=2, command = button_open_gripper)
open_gripper.pack(pady=10)

close_gripper = tk.Button(root, text="Close gripper", width=20, height=2, command = button_close_gripper)
close_gripper.pack(pady=10)


save_button = tk.Button(root, text="Save coords", width=20, height=2, command = save_coords)
save_button.pack(pady=10)



# Timers
firstTimerToggle = False
secondTimerToggle = False

while not stop:
    global camera_frame_coords, camera_frame_angles, frame
    if camera.run() != None:
        camera_frame_coords, camera_frame_angles, frame, isInside, isAtMiddlePoint = camera.run()
    else:
        isInside = False
        isAtMiddlePoint = False
        
    
    # Toggle first timer
    if isInside and not firstTimerToggle:
        time1 = time.time()
        xpos1 = camera_frame_coords[0]
        firstTimerToggle = True
    
    if isAtMiddlePoint and not secondTimerToggle:
        dTime = abs(time.time() - time1)
        xpos2 = camera_frame_coords[1]
        dxpos = abs(xpos1 - xpos2)
        cubeSpeed = dxpos/dTime
        print(cubeSpeed)
        secondTimerToggle = True
        newPosition  = cubeSpeed * 2 # Velocity times 2 seconds = Position of the cube in 2 seconds
        
    
    print(camera_frame_coords, isInside, isAtMiddlePoint)
    root.update_idletasks()
    root.update()
    
        
    if coords_saved > coords_taken:
        break

print("================================================================================")    
print(camera_points)
print("================================================================================")
print(arm_points)
print("================================================================================")

arm_points_array = np.array(arm_points)
camera_points_array = np.array(camera_points)

offset = arm_points_array - camera_points_array
    
print("OFFSETS:", offset)

# Calculate the average offset
average_offset = np.mean(offset, axis=0)
print(average_offset)