import numpy as np
import cv2 as cv
import camera
from camera import *
import tkinter as tk
import gripper

camera_points = []
arm_points = []
coords_saved = 0
stop = False

# Create main window
root = tk.Tk()
root.title("Camera-Robot Calibration")
root.geometry("400x400")  # Adjust size as needed

gripper.connect()
gripper.activate()

def save_coords():
    global coords_saved
    if camera_frame_coords == -1:
            print("Invalid coordinates")
    else:
        print("Coords saved!")
        camera_points.append(camera_frame_coords)
        coords_saved += 1
        
def end_calib():
    global stop
    cv.destroyAllWindows()
    root.destroy()
    stop = True

def button_open_gripper():
    gripper.open_close(POSITION_REQUEST=85, SPEED=10, FORCE=1)
    
def button_close_gripper():
    gripper.open_close(POSITION_REQUEST=0, SPEED=10, FORCE=1)
    
def get_coords():
    coords = entry.get()
    coord_list = list(map(float, coords.split(',')))
    arm_points.append(coord_list)
    entry.delete(0, tk.END)
    print(f"Coordinates saved: {arm_points}")

# Add buttons
save_button = tk.Button(root, text="Save coords from camera", width=20, height=2, command = save_coords)
save_button.pack(pady=10)

stop_button = tk.Button(root, text="Stop Calibration", width=20, height=2, command = end_calib)
stop_button.pack(pady=10)

open_gripper = tk.Button(root, text="Open gripper", width=20, height=2, command = button_open_gripper)
open_gripper.pack(pady=10)

close_gripper = tk.Button(root, text="Close gripper", width=20, height=2, command = button_close_gripper)
close_gripper.pack(pady=10)

get_coords_button = tk.Button(root, text="Save coords", width=20, height=2, command = get_coords)
get_coords_button.pack(pady=10)

entry = tk.Entry(root, width=30)
entry.pack(pady=10)




while not stop:
    global camera_frame_coords, camera_frame_angles, frame, edges
    camera_frame_coords, camera_frame_angles, frame, edges = camera.run()
    
    
    root.update_idletasks()
    root.update()
    
        
    if coords_saved > 5:
        break
    
print("================================================================================")    
print(camera_points)
print("================================================================================")
print(arm_points)
print("================================================================================")

success, rotation_vector, translation_vector = cv.solvePnP(arm_points, camera_points, camera_matrix, distortion_coeffs)

camera_coords = np.array([0, 0, 0])  # Input coordinates in the camera frame

# Convert to robot frame
robot_coords = np.dot(rotation_vector, camera_coords) + translation_vector
print(robot_coords)  # This will give you the robot coordinates corresponding to the camera coordinates