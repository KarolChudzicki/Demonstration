import socket
import time
import math
import gripper
import cv2 as cv
import numpy as np
import ur



#==================== VARIABLES ====================

#==================== GRIPPER SETUP ====================


#==================== CONSTANTS ====================


#TRANSFORMATION MATRIX


#==================== POSITIONS ====================
homePosition = [-0.21, 0.875, -0.075, -1.21, 1.2, 1.21]

pos1 = [0.166, 0.735, 0.35, -1.25, 1.25, 1.2]
pos2 = [0.166, 0.735, 0.35, -1.57, 1.01, 1.49]




#====================CODE====================

#time.sleep(3)
ur.movep(pos2, 0.1, 0.05)

#s.send(b'get_actual_tcp_pose()\n')
#s.send(b'popup(get_actual_tcp_pose(), warning = False, error = False, blocking = False)\n')
#s.send(b'movep(get_actual_tcp_pose(), 0.2, 0.1)\n')

# gripper.connect()
# gripper.activate()

# gripper.close(0xFF, 0xFF, 0x01)



# while True:
        
#     ur.is_steady()
#     if cv.waitKey(1) & 0xFF == ord('q'):
#         break

