import socket
import time
import math
import gripper
import cv2 as cv
import numpy as np

import ur

# ==================== VARIABLES ====================

# ==================== GRIPPER SETUP ====================


# ==================== CONSTANTS ====================


# TRANSFORMATION MATRIX


# ==================== POSITIONS ====================
homePosition = [0.444, 0.801, 0.507, -1.19, -1.21, -1.21]
pos1 = [0.37658, 1.15685, 0.50808, -1.04026, -1.29862, -0.94166]
pos2 = [0.90169, 0.10405, 0.49756, -1.98312, -0.43107, -1.84068]


# ==================== MAIN LOOP ====================
# gripper = gripper.Gripper(port='COM8')
URRobot = ur.URRobot()
# gripper.connect()
# gripper.activate()
#URRobot.movej([-123,-73,111,-36,92,91], 0.1, 0.1)
# time.sleep(5)




