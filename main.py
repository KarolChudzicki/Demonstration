import socket
import time
import math
import gripper
import cv2 as cv
import numpy as np
import camera
from camera import *
import tkinter as tk
import ur
import conveyor_belt

# Classes
URRobot = ur.URRobot()
conveyorBelt = conveyor_belt.conveyorBelt()
gripper = gripper.Gripper()
camera.initSlider()
gripper.activate()
gripper.connect()

# ==================== USER INTERFACE ====================
# Create main window
root = tk.Tk()
root.title("Camera-Robot Calibration")
root.geometry("400x400")

stop = False
is_following_cube = False
cube_coords = []
base_orientation = []
prev_camera_frame_coords = []
prev_camera_frame_angles = []




def go_to_cube():
    global is_following_cube
    is_following_cube = True

def get_new_tool_orientation_and_z():
    global base_orientation
    base_orientation = URRobot.current_Position()[2:]
    print("New base orientation:", base_orientation)
    
def stop_program():
    global stop
    cv.destroyAllWindows()
    root.destroy()
    stop = True

def activate_gripper():
    gripper.activate()
    
def button_open_gripper():
    gripper.open_close(POSITION_REQUEST=85, SPEED=10, FORCE=1)
    
def button_close_gripper():
    gripper.open_close(POSITION_REQUEST=0, SPEED=10, FORCE=1)
    
def run_conveyor():
    conveyorBelt.setDirection(0)
    conveyorBelt.setSpeed(300)
    conveyorBelt.start()

def stop_conveyor():
    conveyorBelt.stop()

def reset():
    firstTimerToggle = False
    secondTimerToggle = False
    

go_button = tk.Button(root, text="Go to cube", width=20, height=2, command = go_to_cube)
go_button.pack(pady=10)

stop_button = tk.Button(root, text="Stop", width=20, height=2, command = stop_program)
stop_button.pack(pady=10)

neworient_button = tk.Button(root, text="New tool TCP and Z", width=20, height=2, command = get_new_tool_orientation_and_z)
neworient_button.pack(pady=10)

act_button = tk.Button(root, text="Activate gripper", width=20, height=2, command = activate_gripper)
act_button.pack(pady=10)

open_gripper = tk.Button(root, text="Open gripper", width=20, height=2, command = button_open_gripper)
open_gripper.pack(pady=10)

close_gripper = tk.Button(root, text="Close gripper", width=20, height=2, command = button_close_gripper)
close_gripper.pack(pady=10)

run_conv = tk.Button(root, text="Run conveyor", width=20, height=2, command = run_conveyor)
run_conv.pack(pady=10)

stop_conv = tk.Button(root, text="Stop conveyor", width=20, height=2, command = stop_conveyor)
stop_conv.pack(pady=10)

reset_values = tk.Button(root, text="Reset values", width=20, height=2, command = reset)
reset_values.pack(pady=10)    

# ==================== POSITIONS ====================
homePosition = [-0.20581, 0.7, -0.07968, -1.18436, -1.20519, -1.21406]
dropPoint = [0.393, 0.741, -0.025, -1.67, -0.85, -1.63]
pos2 = [0.90169, 0.10405, 0.49756, -1.98312, -0.43107, -1.84068]

offset_camera_robot = [0.0721825, 0.8394725]
MAX_Y = 0.9
MIN_Y = 0.8
MAX_X = -0.12
MIN_X = -0.31


def go_to_cube(newPositionX, cubePositionAtMiddleLine):
    global camera_frame_coords, base_orientation
    new_x = round(cubePositionAtMiddleLine[0] + offset_camera_robot[0] + newPositionX,4)
    new_y = round(cubePositionAtMiddleLine[1] + offset_camera_robot[1],4)
            
    if new_x > MAX_X:
        new_x = MAX_X
    if new_x < MIN_X:
        new_x = MIN_X
    if new_y > MAX_Y:
        new_y = MAX_Y
    if new_y < MIN_Y:
        new_y = MIN_Y
            
    cube_coords = [new_x, 
                   new_y + 0.02, 
                   base_orientation[0],
                   base_orientation[1],
                   base_orientation[2],
                   base_orientation[3]]
    print(cube_coords)
    URRobot.movel(cube_coords, 10, 3, 1)
    time.sleep(1)
    gripper.open_close(POSITION_REQUEST=0, SPEED=50, FORCE=1)
    time.sleep(1)
    # Move up 10cm
    cube_coords[2] += 0.1
    URRobot.movel(cube_coords, 0.1, 0.1, 2)
    time.sleep(2)
    URRobot.movel(dropPoint, 0.1, 0.1, 3)
    time.sleep(3)
    gripper.open_close(POSITION_REQUEST=85, SPEED=50, FORCE=1)
    time.sleep(1)
    URRobot.movel(homePosition, 0.1, 0.1, 4)
    time.sleep(4)


# ==================== MAIN LOOP ====================
# gripper = gripper.Gripper(port='COM8')
gripper.open_close(POSITION_REQUEST=85, SPEED=50, FORCE=1)
get_new_tool_orientation_and_z()

# Go to home position
print(homePosition)
URRobot.movel(homePosition, 2, 3, 2)

# Timers
firstTimerToggle = False
secondTimerToggle = False
isInside = False
isAtMiddlePoint = False
grabbingCube = False

while not stop:
    global camera_frame_coords, camera_frame_angles
    root.update_idletasks()
    root.update()
    if camera.run() != None:
        camera_frame_coords, camera_frame_angles, isInside, isAtMiddlePoint = camera.run()
        #print(camera_frame_coords, isInside, isAtMiddlePoint)
    
    if camera_frame_coords == None:
        camera_frame_coords = prev_camera_frame_coords
    else:
        prev_camera_frame_coords = camera_frame_coords
    
    
    #Toggle first timer
    
    if isInside == True and firstTimerToggle == False:
        time1 = time.time()
        xpos1 = camera_frame_coords[0]
        firstTimerToggle = True
        print("Time1", time1)
        
    if isAtMiddlePoint == True and secondTimerToggle == False:
        dTime = abs(time.time() - time1)
        xpos2 = camera_frame_coords[1]
        dxpos = abs(xpos1 - xpos2)
        cubeSpeed = round(dxpos/dTime,4)
        newPositionX = round(cubeSpeed * 2, 4) # Velocity times 2 seconds = Position of the cube in 2 seconds
        print("Cube speed: ",cubeSpeed, "X pos inc: ", newPositionX)
        secondTimerToggle = True
        cubePositionAtMiddleLine = camera_frame_coords    
        
    
    
    
    
    if firstTimerToggle and secondTimerToggle:
        go_to_cube(newPositionX, cubePositionAtMiddleLine)
        firstTimerToggle = False
        secondTimerToggle = False





