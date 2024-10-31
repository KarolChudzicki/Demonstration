import socket
import math

# REMEMBER TO CHANGE IP ADDRESS IN THE PROPORTIES OF THE DEVICES CONNECTED VIA ETHERNET (!!! TO A DIFFERENT THAN HOST1 !!!)
HOST1 = '192.38.66.227'        # UR5
PORT1 = 30003            # The same port as used by the server
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST1, PORT1))

#==================== FUNCTIONS ====================
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
    
def is_steady():
    command = 'is_steady()\n'
    s.send(command.encode('utf-8'))
    response = s.recv(1)
    print(response)