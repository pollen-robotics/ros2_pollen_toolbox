import time

import numpy as np
from reachy2_sdk import ReachySDK


def gripper_test():
    print("Trying to connect on localhost Reachy...")
    time.sleep(1.0)
    reachy = ReachySDK(host="localhost")
    print(reachy.joints)
    time.sleep(1.0)
    if reachy._grpc_status == "disconnected":
        print("Failed to connect to Reachy, exiting...")
        return
    print(reachy.r_arm)
    reachy.turn_on()
    while True:
        print("Closing gripper")
        reachy.r_arm.gripper.close()
        reachy.l_arm.gripper.close()
        
        # reachy.r_arm.gripper.set_opening(10)
        # reachy.l_arm.gripper.set_opening(10)
        print(f"r={reachy.r_arm.gripper}")
        print(f"l={reachy.l_arm.gripper}")

        time.sleep(2.0)
        print("Opening gripper")

        reachy.r_arm.gripper.open()
        reachy.l_arm.gripper.open()
        print(f"r={reachy.r_arm.gripper}")
        print(f"l={reachy.l_arm.gripper}")
        time.sleep(2.0)

        # reachy.r_arm.gripper.set_opening(75)
        # reachy.l_arm.gripper.set_opening(75)
        # print(f"r={reachy.r_arm.gripper}")
        # print(f"l={reachy.l_arm.gripper}")


if __name__ == "__main__":
    # main_test()
    gripper_test()
