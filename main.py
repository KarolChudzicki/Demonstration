import socket
import time
import math
import gripper
import cv2 as cv
import numpy as np



# ==================== VARIABLES ====================

# ==================== GRIPPER SETUP ====================


# ==================== CONSTANTS ====================


# TRANSFORMATION MATRIX


# ==================== POSITIONS ====================
homePosition = [-0.21, 0.875, -0.075, -1.21, 1.2, 1.21]

pos1 = [0.166, 0.735, 0.35, -1.25, 1.25, 1.2]
pos2 = [0.166, 0.735, 0.35, -1.57, 1.01, 1.49]




# ==================== MAIN LOOP ====================
gripper = gripper.Gripper(port='COM8')
#print(ur.currentPos())
gripper.connect()
gripper.activate()
#gripper.open_close(40, 0xFF, 0x01)

gripper.open_close(POSITION_REQUEST=2)

#time.sleep(3)
#ur.movep(pos2, 0.1, 0.05)

