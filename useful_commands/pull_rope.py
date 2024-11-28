from reachy2_sdk import ReachySDK
from reachy2_sdk.utils.utils import rotate_in_self
import time

# IP = "10.0.0.253"
IP = "192.168.50.71"

# The mobile base is not available in fake mode (rviz visualization)
fake = IP == "localhost"


reachy = ReachySDK(IP)
if not fake:
    reachy.mobile_base.reset_odometry()
    # Uncomment the two lines below to deactivate the mobile base in order to move the robot freely where you want
    # reachy.mobile_base.turn_off()
    # exit()

reachy.turn_on()

# Look to the left
reachy.head.goto([0, 0, 50])


reachy.l_arm.goto_posture("elbow_90", wait=False)
reachy.r_arm.goto_posture("elbow_90", wait=True)

right_start_pose = reachy.r_arm.forward_kinematics()
left_start_pose = reachy.l_arm.forward_kinematics()


reachy.r_arm.gripper.open()
reachy.l_arm.gripper.open()
# Start the pull rope movement

# making the extended position for the right arm with a rotation
right_extend = right_start_pose.copy()
right_extend[:3, 3] += [0.1, 0.2, 0.0]

right_extend = rotate_in_self(right_extend, [0, 60, -90])
# move the arm to the extended position. Wait for the movement to finish
reachy.r_arm.goto(right_extend, duration=1.5, wait=True)

reachy.r_arm.gripper.close()
time.sleep(0.2)

left_extend = left_start_pose.copy()
left_extend[:3, 3] += [0.1, -0.2, 0.0]
left_extend = rotate_in_self(left_extend, [0, 60, 90])

# Move the mobile base asynchronously.
# TODO this should not be necessary in the future, as we will add a non blocking goto for the mobile base
# For now, we run it in a thread so that the next command is executed at the same time

# At the same time, move the left arm to the extended position with rotation
# goto_id = reachy.l_arm.goto(left_extend, duration=2.0, wait=False)

# Translate the arm
reachy.r_arm.translate_by(-0.2, 0, 0, duration=1.5, wait=True)


    # for _ in range(2):
reachy.l_arm.gripper.close()
time.sleep(0.2)

# reachy.l_arm.translate_by(-0.2, 0, 0, duration=1.5, wait=False)


reachy.r_arm.gripper.open()

while True :
    reachy.r_arm.send_cartesian_interpolation(
        right_extend, duration=1.3, arc_direction="right", precision_distance_xyz=0.1
    )

    reachy.r_arm.send_cartesian_interpolation(
        right_start_pose, duration=1.3, arc_direction="right", precision_distance_xyz=0.1
    )



reachy.r_arm.gripper.close()
# time.sleep(0.2)


# goto_id = reachy.r_arm.translate_by(-0.2, 0, 0, duration=1.5)

# reachy.l_arm.gripper.open()
# reachy.l_arm.send_cartesian_interpolation(
#     left_extend, duration=1.3, arc_direction="left", precision_distance_xyz=0.1
# )