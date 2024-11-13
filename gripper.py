import serial #pyserial
import time
import binascii
import misc


counter = 0
gripper_opening = 85

def connect():
    global ser
    ser = serial.Serial(port='COM8', baudrate=115200, timeout=1, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=8)



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
def close(POSITION_REQUEST, SPEED, FORCE):
    data = ser.readline()
    SLAVE_ID = 0x09
    FUNCTION_CODE = 0x10
    FIRST_REGISTER_ADDRESS_HIGH = 0x03
    FIRST_REGISTER_ADDRESS_LOW = 0xE8
    REGISTERS_WRITTEN_TO_HIGH = 0x00
    REGISTERS_WRITTEN_TO_LOW = 0x03
    NUMBER_OF_DATA_BYTES = 0x06
    ACTION_REQUEST = 0x09
    GRIPPER_OPTIONS1 = 0x00
    GRIPPER_OPTIONS2 = 0x00
    
    command = [SLAVE_ID, 
               FUNCTION_CODE,
               FIRST_REGISTER_ADDRESS_HIGH, 
               FIRST_REGISTER_ADDRESS_LOW,
               REGISTERS_WRITTEN_TO_HIGH,
               REGISTERS_WRITTEN_TO_LOW, 
               NUMBER_OF_DATA_BYTES, 
               ACTION_REQUEST, 
               GRIPPER_OPTIONS1, 
               GRIPPER_OPTIONS2, 
               POSITION_REQUEST, 
               SPEED, 
               FORCE]
    

    CRCL, CRCH, _ =  misc.modbusCrc(command)
    command.append(CRCH)
    command.append(CRCL)
    ser.write(command)
    ser.readline()
    
    while True:
        status = check_status()
        if status["OBJECT_DETECTION_STATUS"] == 3:
            print("Kid named finger")
            break
        




def open(POSITION_REQUEST, SPEED, FORCE):
    data = ser.readline()
    ACTION_REQUEST = 0x09
    GRIPPER_OPTIONS1 = 0x00
    GRIPPER_OPTIONS2 = 0x00
    
    command = [0x09, 0x10, 0x03, 0xE8, 0x00, 0x03, 0x06, ACTION_REQUEST, GRIPPER_OPTIONS1, GRIPPER_OPTIONS2, POSITION_REQUEST, SPEED, FORCE]
    CRCL, CRCH, _ = misc.modbusCrc(command)
    
    command.append(CRCH)
    command.append(CRCL)
    ser.write(command)
    ser.readline()
    
    while True:
        status = check_status()
        if status["OBJECT_DETECTION_STATUS"] == 3:
            print("Kid named finger")
            break


def check_status():
    SLAVE_ID = 0x09
    FUNCTION_CODE = 0x03 # Read holding registers
    FIRST_REGISTER_ADDRESS_HIGH = 0x07
    FIRST_REGISTER_ADDRESS_LOW = 0xD0
    REGISTERS_WRITTEN_TO_HIGH = 0x00
    REGISTERS_WRITTEN_TO_LOW = 0x03
    
    command = [SLAVE_ID, 
               FUNCTION_CODE,
               FIRST_REGISTER_ADDRESS_HIGH, 
               FIRST_REGISTER_ADDRESS_LOW,
               REGISTERS_WRITTEN_TO_HIGH,
               REGISTERS_WRITTEN_TO_LOW]
    
    CRCL, CRCH, _ =  misc.modbusCrc(command)
    command.append(CRCH)
    command.append(CRCL)
    ser.write(command)
    
    data = ser.readline()
    data = [hex(byte) for byte in data]
    
    # Get useful values of the gripper
    GRIPPER_STATUS_BYTE = bin(int(data[3],16))
    GRIPPER_STATUS_PER_BIT = [int(bit) for bit in GRIPPER_STATUS_BYTE[2:]]
    POSITION = int(data[6],16)
    FINGER_CURRENT = (int(data[8],16) << 8) | (int(data[9],16)) # Merge two hex values together
    
    # Explanation of each bit
    ACTIVATION_STATUS = GRIPPER_STATUS_PER_BIT[7] # 0 - gripper reset, 1 - gripper activation
    ACTION_STATUS = GRIPPER_STATUS_PER_BIT[4] # 0 - stopped, 1 - go to position request
    GRIPPER_STATUS = GRIPPER_STATUS_PER_BIT[3] + GRIPPER_STATUS_PER_BIT[2] * 2 # 0 - gripper is in reset, 1 - activation in progress, 2 - not used, 3 - activation completed
    OBJECT_DETECTION_STATUS = GRIPPER_STATUS_PER_BIT[1] + GRIPPER_STATUS_PER_BIT[0] * 2 # 0 - fingers are in motion towards position, 1 - fingers stopped due to contact during opening, 2 - fingers stopped due to contact during closing, 3 - fingers are at requested position
    
    # Returns a dictionary with all parameters of the gripper
    return {
        "ACTIVATION_STATUS": ACTIVATION_STATUS,
        "ACTION_STATUS": ACTION_STATUS,
        "GRIPPER_STATUS": GRIPPER_STATUS,
        "OBJECT_DETECTION_STATUS": OBJECT_DETECTION_STATUS,
        "POSITION": POSITION,
        "FINGER_CURRENT": FINGER_CURRENT
    }
    

def pos_to_mm(position):
    if position > 85 or position < 0:
        print("ERROR: Given gripper position is outside the possible range")
        return 0
    else:
        return (round(position * 255.0/85.0))
    
    
    