from pymodbus.client import ModbusSerialClient as ModbusClient

client = ModbusClient(port='COM10', baudrate=9600, timeout=1)


if client.connect():
    print("Connected to Modbus device.")

    
    result = client.write_register(0x0106, 0x0001, 0x0001)
    print("Write command result:", result)

    result = client.write_register(0x0064, 0xFF00, 0x01)
    print("Write command result:", result)
    
    result = client.read_holding_registers(0x0000, 1, 0x01)
    print("Read result", result)

    client.close()
else:
    print("Failed to connect to Modbus device.")