import time

import numpy as np
from reachy2_sdk import ReachySDK


def gripper_test():
    print("Trying to connect on localhost Reachy...")
    reachy = ReachySDK(host="localhost")
    reachy.r_arm.gripper.close()
    reachy.l_arm.gripper.close()

if __name__ == "__main__":
    # main_test()
    gripper_test()
