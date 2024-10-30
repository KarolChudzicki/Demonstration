import socket
import time
import math
import gripper
import cv2 as cv
import numpy as np
#import camera

print("Start : %s") # % time.ctime()

HOST1 = '192.38.66.227'        # UR5
PORT1 = 30003              # The same port as used by the server
#s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#s.connect((HOST1, PORT1))

#==================== VARIABLES/ ====================
xy = 0
corners = 0
#====================GRIPPER SETUP====================


#====================CONSTANTS====================
#ORIGIN POINTS (X,Y,Z,RX,RY,RZ)
#P0 = [300, 145, -670, 2.43, -1.98, 0]
#PX = [250, 145, -670, 2.43, -1.98, 0]
#PY = [300, 120, -670, 2.43, -1.98, 0]
xyZero = [0, 195]


#TRANSFORMATION MATRIX
R = [[-1, 0, 0], [0, -1, 0], [0, 0, 1]]

#TOOL POSITIONS
home = [300,150,-700,-3.14,0,0]

#JOINT POSITIONS
pos1 = [1.2, -1.48, -1.92, -3.22, -1.57, 0.5]
pos2 = [1.2, -1.48, -1.92, -3.22, -1.57, 0.0]

pos3 = [-11.5,-85,-50,-135,-90,0]
pos4 = [-11.5,-85,-50,-125,-80,30]

pos5 = [0.300, -0.170, 1.072, 0, 0, 1.37]
pos6 = [0.200, -0.170, 1.072, 0, 0, 1.37]
pos7 = [0.300, -0.070, 1.072, 0, 0, 1.37]

zeroPos = [0.044, -0.300, 1.110, 0, 0, 0]
zeroPosUp = [0.044, -0.300, 0.9, 0, 0, 0]
boxUp = [0.350, -0.275, 0.9, 0, 0, 0]
boxDown = [0.350, -0.275, 0.8, 0, 0, 0]
#====================FUNCTIONS====================
#Joint position in deg
def movej(position, acc, vel):
    position = [x * math.pi/180.0 for x in position]
    command = 'movej(' + str(position) + ',' + str(acc) + ',' + str(vel) + ')\n'
    s.send(command.encode('utf-8'))

def movep(position, acc, vel):
    command = 'movep(p' + str(position) + ',' + str(acc) + ',' + str(vel) + ')\n'
    s.send(command.encode('utf-8'))

def movel(position, acc, vel, t):
    command = 'movel(p' + str(position) + ',' + str(acc) + ',' + str(vel) + ',' + str(t) + ')\n'
    s.send(command.encode('utf-8'))

#Joint speed in radians
def speed(jointSpeed):
    command = 'speed(' + str(jointSpeed) + ')\n'
    s.send(command.encode('utf-8'))

def rSleep(time):
    command = 'sleep(' + str(time) + '.)\n'
    s.send(command.encode('utf-8'))


#====================CODE====================
#movep(zeroPosUp, 0.2, 0.15)
#s.send(b'get_actual_tcp_pose()\n')
#s.send(b'popup(get_actual_tcp_pose(), warning = False, error = False, blocking = False)\n')
#s.send(b'movep(get_actual_tcp_pose(), 0.2, 0.1)\n')
gripper.connect()
gripper.activate()
print("GRIPPER ACTIVE")
gripper.close(0xFF, 0xFF, 0x00)
#gripper.open(0x00, 0xFF, 0xFF)
#gripper.close(0x4F, 0xFF, 0x10)
#gripper.open(0x00, 0xFF, 0xFF)



# while True:
        
    
#     if cv.waitKey(1) & 0xFF == ord('q'):
#         break


