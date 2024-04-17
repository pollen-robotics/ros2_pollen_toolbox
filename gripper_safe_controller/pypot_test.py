import pypot.dynamixel
import time 
import math

#ports = pypot.dynamixel.get_available_ports()
#dxl_io = pypot.dynamixel.DxlIO(ports[0], baudrate=1000000)

dxl_io = pypot.dynamixel.DxlIO("/dev/reachy2_right_arm", baudrate=2000000)

#print("Connected. Scanning...")
#list_of_ids = dxl_io.scan()
#print(list_of_ids)
i = 0
loads = []
while True :
    val =65 +60*math.sin(time.time()*2)
    dxl_io.set_goal_position({43:val})
    i+=1
    loads.append(dxl_io.get_present_load([43])[0])
    if i%10 == 0:
        #print(loads)
        sum = 0
        for l in loads:
            sum+=l
        sum = sum/len(loads)
        print(f"avg_load={sum:.1f}")
        loads = []
    

    time.sleep(0.01)
