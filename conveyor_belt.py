import serial
import misc
import struct
import time
from pymodbus.client import ModbusSerialClient as ModbusClient
# ==================== VARIABLES ====================
SLAVE_ADDRESS = 0x01

FORWARD_REGISTER = 0x0049  # Register for forward operation
STOP_REGISTER = 0x004B  # Register for stop operation
START_VALUE = 0xFF00  # Start value
STOP_VALUE = 0xFF00  # Stop value


# ==================== CONNECTION TO THE DEVICE VIA SERIAL ====================
def connect():
    global ser
    ser = serial.Serial(port='COM10', baudrate=115200, timeout=1, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)
    

# ==================== FUNCTION FOR BUILDING MODBUS FRAME ====================
def modbus_frame(slave_address, function, register_address, value_to_write):
    frame = struct.pack('>B', slave_address)     #big endian unsigned byte format
    frame += struct.pack('>B', function)         #big endian unsigned byte format
    frame += struct.pack('>H', register_address) #big endian unsigned short format
    frame += struct.pack('>H', value_to_write)   #big endian unsigned short format
    
    frame_without_crc = struct.pack('>BBHH', slave_address, function, register_address, value_to_write)

    # calculation CRC
    crcl, crch, crc = calculate_crc(frame_without_crc)
    print(crcl, crch, crc)
    
    frame_without_crc += struct.pack('<H', crc)
    print(struct.pack('<H', crch))
    print(struct.pack('<H', crcl))
    print(frame_without_crc)
    return frame_without_crc

def calculate_crc(data):
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for _ in range(8):
            if (crc & 1):
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    crcl = (crc >> 8) & 0xFF
    crch = crc & 0xFF
    return crcl, crch, crc



connect()
freq_frame = modbus_frame(0x01, 0x06, 0x0201, 0x0010)


comm = b'\x01\x06\x00\x01\x00\x01\x48\x0A'
commando = b'\x01\x06\x02\x01\x00\x10\x7E\x78'
commando1 = b'\x01\x05\x00\x48\xFF\x00\x2C\x84'
commando2 = b'\x01\x05\x00\x48\xFF\x00\x9C\x7D'
print(ser.readline())
ser.write(comm)
print(ser.readline())
ser.write(commando)
print(ser.readline())
ser.write(commando1)
print(ser.readline())
time.sleep(1)
ser.write(commando2)
print(ser.readline())

