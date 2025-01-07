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
#URRobot = ur.URRobot()
conveyorBelt = conveyor_belt.conveyorBelt()
gripper = gripper.Gripper()
camera.initSlider()
gripper.activate()
gripper.connect()

# ==================== USER INTERFACE ====================
# Create main window
root = tk.Tk()
root.title("Camera-Robot Calibration")
root.geometry("400x600")

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
    base_orientation = 0 #URRobot.current_Position()[2:]
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
    
def run_conveyor100():
    conveyorBelt.setDirection(0)
    conveyorBelt.setSpeed(100)
    conveyorBelt.start()
    
def run_conveyor200():
    conveyorBelt.setDirection(0)
    conveyorBelt.setSpeed(200)
    conveyorBelt.start()
    
def run_conveyor300():
    conveyorBelt.setDirection(0)
    conveyorBelt.setSpeed(300)
    conveyorBelt.start()

def stop_conveyor():
    conveyorBelt.stop()

    

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

run_conv100 = tk.Button(root, text="Run conveyor 100", width=20, height=2, command = run_conveyor100)
run_conv100.pack(pady=10)

run_conv200 = tk.Button(root, text="Run conveyor 200", width=20, height=2, command = run_conveyor200)
run_conv200.pack(pady=10)

run_conv300 = tk.Button(root, text="Run conveyor 300", width=20, height=2, command = run_conveyor300)
run_conv300.pack(pady=10)

stop_conv = tk.Button(root, text="Stop conveyor", width=20, height=2, command = stop_conveyor)
stop_conv.pack(pady=10)

# ==================== POSITIONS ====================
homePosition = [-0.470, 0.708, -0.084, -1.18, -1.21, -1.1958]
homePositionVertical = [-0.25, 0.77, -0.13, 0.023, -3.1394, -0.0165]
dropPoint = [0.393, 0.741, -0.025, -1.67, -0.85, -1.63]
dropPointVertical = [0.315, 0.625, -0.13, 0.023, -3.1394, -0.0165]


offset_camera_robot = [-0.3011275, 0.8875]
MAX_Y = 0.9
MIN_Y = 0.8
MAX_X = 0.1
MIN_X = -0.250


def go_to_cube(newPositionX, cubePositionAtMiddleLine, tcubeHome, angleOfCube):
    global camera_frame_coords, base_orientation
    new_x = round(cubePositionAtMiddleLine[0] + offset_camera_robot[0] + newPositionX,4)
    new_y = round(cubePositionAtMiddleLine[1] + offset_camera_robot[1],4)
    new_angle = float(base_orientation[1] - round(np.deg2rad(angleOfCube-90),2))
    print("NEW angle: ",new_angle)
            
    if new_x > MAX_X:
        new_x = MAX_X
    if new_x < MIN_X:
        new_x = MIN_X
    if new_y > MAX_Y:
        new_y = MAX_Y
    if new_y < MIN_Y:
        new_y = MIN_Y
            
    cube_coords = [new_x, 
                   new_y - 0.085, 
                   -0.175,
                   new_angle,
                   base_orientation[2],
                   base_orientation[3]]
    print(cube_coords)
    # time.sleep(tcubeHome)
    # timeToCatchCube = 3 - tcubeHome
    # URRobot.movel(cube_coords, 10, 10, timeToCatchCube)
    # time.sleep(timeToCatchCube - 0.1)
    # gripper.open_close(POSITION_REQUEST=0, SPEED=100, FORCE=1)
    # time.sleep(1)
    # # Move up 10cm
    # cube_coords[2] += 0.1
    # URRobot.movel(cube_coords, 0.1, 0.1, 2)
    # time.sleep(2)
    # URRobot.movel(dropPointVertical, 0.1, 0.1, 3)
    # time.sleep(3)
    # gripper.open_close(POSITION_REQUEST=85, SPEED=100, FORCE=1)
    # time.sleep(1)
    # URRobot.movel(homePositionVertical, 0.1, 0.1, 4)
    # time.sleep(4)


# ==================== MAIN LOOP ====================
# gripper = gripper.Gripper(port='COM8')
gripper.open_close(POSITION_REQUEST=85, SPEED=50, FORCE=1)


# Go to home position
#URRobot.movel(homePositionVertical, 2, 3, 3)

#Angles
angle_sum = 0
angle_length = 0

# Timers
firstTimerToggle = False
secondTimerToggle = False
isAtFirstLine = False
isAtSecondLine = False
grabbingCube = False
time.sleep(5)
get_new_tool_orientation_and_z()

while not stop:
    global camera_frame_coords, camera_frame_angles
    root.update_idletasks()
    root.update()
    
    camera_frame_coords, camera_frame_angles, isAtFirstLine, isAtSecondLine = camera.run()

    
        
    
    if camera_frame_coords == None:
        camera_frame_coords = prev_camera_frame_coords
    else:
        prev_camera_frame_coords = camera_frame_coords
        
        if isAtFirstLine and not isAtSecondLine and camera_frame_angles != None:
            angle_sum += camera_frame_angles
            angle_length += 1
        
        
        #Toggle first timer
    
        if isAtFirstLine == True and firstTimerToggle == False:
            time1 = time.time()
            xpos1 = camera_frame_coords[0]
            firstTimerToggle = True
            print("Time1", time1)
            
            
        if isAtSecondLine == True and secondTimerToggle == False: 
            dTime = abs(time.time() - time1)
            xpos2 = camera_frame_coords[1]
            dxpos = abs(xpos1 - xpos2)
            cubeSpeed = round(dxpos/dTime,4)
            newPositionX = round(cubeSpeed * 3, 4) # Velocity times 2 seconds = Position of the cube in 2 seconds
            print("Cube speed: ",cubeSpeed, "DTIME: ", dTime)
            secondTimerToggle = True
            cubePositionAtMiddleLine = camera_frame_coords   
            
            angleOfCube = angle_sum/angle_length
            angle_sum = 0
            angle_length = 0
            
            # Time for the cube to reach home pos
            tcubeHome = (homePositionVertical[0] - (xpos2 + offset_camera_robot[0]))/cubeSpeed
            
    
    
     
        
    
    
    
    
    if firstTimerToggle and secondTimerToggle:
        print(angleOfCube)
        #go_to_cube(newPositionX, cubePositionAtMiddleLine, tcubeHome, angleOfCube)
        time.sleep(10)
        firstTimerToggle = False
        secondTimerToggle = False





