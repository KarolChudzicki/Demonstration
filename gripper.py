import serial
import time
import binascii



ser = serial.Serial(port='COM8', baudrate=115200, timeout=1, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=8)

counter = 0
gripper_opening = 85



def activate():
    ser.write(b'\x09\x10\x03\xE8\x00\x03\x06\x00\x00\x00\x00\x00\x00\x73\x30')
    data = ser.readline()
    print("Response 1", data)
    time.sleep(0.01)
    
    ser.write(b'\x09\x10\x03\xE8\x00\x03\x06\x01\x00\x00\x00\x00\x00\x72\xE1')
    data = ser.readline()
    print("Response 2", data)
    time.sleep(0.01)

    while True:
        ser.write(b'\x09\x03\x07\xD0\x00\x01\x85\xCF')
        data = ser.readline()
        print("Response 3", data)
        time.sleep(0.5)
        if data == b'\x09\x03\x02\x31\x00\x4C\x15':
            break

#============COMMANDS============
def modbusCrc(msg:str) -> int:
    crc = 0xFFFF
    for n in range(len(msg)):
        crc ^= msg[n]
        for i in range(8):
            if crc & 1:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    
    crcl = (crc >> 8) & 0xFF
    crch = crc & 0xFF
    return crcl, crch


def close(POSITION_REQUEST, SPEED, FORCE):
    data = ser.readline()
    ACTION_REQUEST = 0x09
    GRIPPER_OPTIONS1 = 0x00
    GRIPPER_OPTIONS2 = 0x00
    
    command = [0x09, 0x10, 0x03, 0xE8, 0x00, 0x03, 0x06, ACTION_REQUEST, GRIPPER_OPTIONS1, GRIPPER_OPTIONS2, POSITION_REQUEST, SPEED, FORCE]
    

    CRCL, CRCH =  modbusCrc(command)
    command.append(CRCH)
    command.append(CRCL)
    print(command)
    ser.write(command)
    data = ser.readline()
    
    while True:
        ser.write(b'\x09\x03\x07\xD0\x00\x03\x04\x0E')
        data = ser.readline()
        data = [hex(byte) for byte in data]
        first_two = int(data[3], 16)
        first_two = (first_two & 0b11000000) >> 6
        
        if first_two != 0b00:
            break
        
    print(data)




def open(POSITION_REQUEST, SPEED, FORCE):
    data = ser.readline()
    ACTION_REQUEST = 0x09
    GRIPPER_OPTIONS1 = 0x00
    GRIPPER_OPTIONS2 = 0x00
    CRC1 = 0x72
    CRC2 = 0x19
    command = [0x09, 0x10, 0x03, 0xE8, 0x00, 0x03, 0x06, ACTION_REQUEST, GRIPPER_OPTIONS1, GRIPPER_OPTIONS2, POSITION_REQUEST, SPEED, FORCE, CRC1, CRC2]
    ser.write(command)
    ser.readline()
    
    while True:
        ser.write(b'\x09\x03\x07\xD0\x00\x03\x04\x0E')
        data = ser.readline()
        data = [hex(byte) for byte in data]
        first_two = int(data[3], 16)
        first_two = (first_two & 0b11000000) >> 6
        
        if first_two != 0b00:
            break




def pos_to_mm(position):
    if position > 85 or position < 0:
        print("ERROR: Given gripper position is outside the possible range")
        return 0
    else:
        return (round(position * 255.0/85.0))
    
    
    
    
    
    
    
    
    
    
    
    '''
    while True:
        time.sleep(0.01)
        #ser.write(b'\x09\x03\x07\xD0\x00\x03\x04\x0E')
        #ser.write(b'\x09\x10\x03\xE9\x00\x02\x04\x60\xE6\x3C\xC8\xEC\x7C')
        #ser.write(b'\x09\x17\x07\xD0\x00\x02\x03\xE9\x00\x02\x04\x00\xE6\x3C\xC8\x2D\x0C')
        ser.write(b'\x09\x04\x03\xE9\x00\x03\xEC\x7C')
        data = ser.readline()
        data = [hex(byte) for byte in data]
        #first_two = int(data[3], 16)
        #first_two = (first_two & 0b11000000) >> 6
        print(data)
        #if first_two != 0b00:
        #    break
        break
    '''