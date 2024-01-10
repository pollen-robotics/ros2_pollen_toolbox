import pypot.dynamixel
import time 
import math

ports = pypot.dynamixel.get_available_ports()
dxl_io = pypot.dynamixel.DxlIO(ports[0], baudrate=1000000)

print("Connected. Scanning...")
list_of_ids = dxl_io.scan()
print(list_of_ids)


