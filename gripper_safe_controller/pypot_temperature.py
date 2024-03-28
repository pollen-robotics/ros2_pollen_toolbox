import math
import time

import pypot.dynamixel

ports = pypot.dynamixel.get_available_ports()
dxl_io = pypot.dynamixel.DxlIO(ports[0], baudrate=1000000)

id = 11
dxl_io.enable_torque([id])
target_error = 3
dt = 0.1
t0 = time.time()
last_t_int = 0
try:
    while True:
        t = time.time() - t0
        pos = dxl_io.get_present_position([id])[0]
        temp = dxl_io.get_present_temperature([id])[0]
        goal = pos - target_error
        dxl_io.set_goal_position({id: goal})
        # print the temperature every second
        if int(t) > last_t_int:
            last_t_int = int(t)
            print(
                f"Temperature: {temp}°C, Position: {pos}°, Goal: {goal}°, Time: {t:.2f}s"
            )

        time.sleep(dt)
finally:
    dxl_io.set_goal_position({id: 120})
