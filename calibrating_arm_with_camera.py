import numpy as np
import cv2 as cv
import camera
from camera import *
import tkinter as tk
import gripper

camera_coords = []
arm_coords = []
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
        camera_coords.append(camera_frame_coords)
        coords_saved += 1
        
def end_calib():
    global stop
    cv.destroyAllWindows()
    root.destroy()
    stop = True

def button_open_gripper():
    gripper.open(0x00, 0xFF, 0x01)
    
def button_close_gripper():
    gripper.close(0xFF, 0xFF, 0x01)
    
def get_coords():
    coords = entry.get()
    coord_list = list(map(float, coords.split(',')))
    arm_coords.append(coord_list)
    entry.delete(0, tk.END)
    print(f"Coordinates saved: {arm_coords}")

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
print(camera_coords)
print("================================================================================")
print(arm_coords)
print("================================================================================")