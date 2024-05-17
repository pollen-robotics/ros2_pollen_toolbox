import time

import numpy as np
from google.protobuf.wrappers_pb2 import FloatValue
from reachy2_sdk import ReachySDK
from reachy2_sdk_api.arm_pb2 import ArmCartesianGoal
from reachy2_sdk_api.kinematics_pb2 import Matrix4x4
from scipy.spatial.transform import Rotation

reachy = ReachySDK(host="localhost")

# reachy.r_gripper._gripper_stub.TurnOn(reachy.r_gripper.part_id)

# reachy.r_arm._arm_stub.TurnOn(reachy.r_arm.part_id)
reachy.turn_on()


# add hints
def goto(x: float, y: float, z: float, partid=1) -> None:
    goal = np.array(
        [
            [0, 0, 1, x],
            [0, 1, 0, y],
            [1, 0, 0, z],
            # [1, 0, 0, x],
            # [0, 0, -1, y],
            # [0, 1, 0, z],
            [0, 0, 0, 1],
        ]
    )
    # target = ArmCartesianGoal(
    #     id=reachy.r_arm.part_id,
    #     goal_pose=goal,
    #     duration=FloatValue(value=1.0),
    # )

    target = ArmCartesianGoal(
        id={"id": partid, "name": "r_arm"},
        goal_pose=Matrix4x4(data=goal.flatten().tolist()),
        duration=FloatValue(value=1.0),
    )
    reachy.r_arm._arm_stub.SendArmCartesianGoal(target)

def goto_xyzrpy(x,y,z,roll,pitch,yaw, partid):
    R = Rotation.from_euler('xyz', [roll,pitch,yaw], degrees=True)
    goal = np.zeros((4,4))
    goal[:3,:3] = R.as_matrix()
    goal[:,3] = np.array([x,y,z, 1])

    # print(goal)

    target = ArmCartesianGoal(
        id={"id": partid, "name": "r_arm"},
        goal_pose=Matrix4x4(data=goal.flatten().tolist()),
        duration=FloatValue(value=1.0),
    )
    reachy.r_arm._arm_stub.SendArmCartesianGoal(target)



goto(1, 1, 0)

print("coucou")
fixed_x = .3  # Fixed x-coordinate
center_y, center_z = 0, 0  # Center of the circle in y-z plane
num_steps = 200  # Number of steps to complete the circle
frequency = 50  # Update frequency in Hz
# frequency = 10  # Update frequency in Hz
step = 0  # Current step
circle_period = 3
# with open(self.fifo_path, "w") as fifo:
t0 = time.time()
last = t0
y,z = -0.2,0.2

# x,y,z,r,p,yaw
pose1 = [0.3, -0.4, -0.3, 0.0,-30,0.0]
pose2 = [0.3, -0.4, -0.3, 0.0,0,0.0]
duration = 3
goto1 = True
while True:

    # cycle = time.time() - last
    # if cycle > 10:
    #     y *= -1
    #     z *= -1
    #     last = time.time()
    #     print('switch', 'y(2)', y, 'z(1)', z)

    # Call the goto function with the constant x and calculated y, z coordinates
    # goto(fixed_x, center_y, z, partid=2)
    # goto(fixed_x, y, center_z, partid=1)
    elapsed = time.time()-last
    if elapsed > duration:
        last = time.time()
        goto1 ^=1

    if goto1:
        goto_xyzrpy(*pose1, partid=1)
    else:
        goto_xyzrpy(*pose2, partid=1)


    # goto_xyzrpy(*pose2, partid=1)
    time.sleep(0.01)
