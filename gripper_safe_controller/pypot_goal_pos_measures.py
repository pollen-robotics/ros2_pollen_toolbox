import math
import time

import pypot.dynamixel

ports = pypot.dynamixel.get_available_ports()
dxl_io = pypot.dynamixel.DxlIO(ports[0], baudrate=1000000)

id = 11
dxl_io.enable_torque([id])
target_error = 5
dt = 0.01
t0 = time.time()
start_pos = 38
prev_goal = start_pos
max_error = 26
try:
    while True:
        t = time.time() - t0
        pos = dxl_io.get_present_position([id])[0]
        temp = dxl_io.get_present_temperature([id])[0]
        goal = start_pos - int(t / 2) * 1
        dxl_io.set_goal_position({id: goal})
        # print the temperature every second
        if prev_goal != goal:
            prev_goal = goal
            print(
                f"Temperature: {temp}°C, Position: {pos}°, Goal: {goal}°, Time: {t:.2f}s"
            )
        if abs(goal - start_pos) > max_error:
            print("End of test")
            continue

        time.sleep(dt)
finally:
    dxl_io.set_goal_position({id: 60})
