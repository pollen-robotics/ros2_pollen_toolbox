import time

import numpy as np
from google.protobuf.wrappers_pb2 import FloatValue
from reachy2_sdk_api.arm_pb2 import ArmCartesianGoal
from reachy2_sdk_api.kinematics_pb2 import Matrix4x4

from reachy2_sdk import ReachySDK

reachy = ReachySDK(host="localhost")

# reachy.r_gripper._gripper_stub.TurnOn(reachy.r_gripper.part_id)

reachy.r_arm._arm_stub.TurnOn(reachy.r_arm.part_id)


# add hints
def goto(x: float, y: float, z: float, partid=1) -> None:
    goal = np.array(
        [
            [0, 0, 1, x],
            [0, 1, 0, y],
            [1, 0, 0, z],
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


goto(1, 1, 0)

print("coucou")
radius = 0.5  # Circle radius
fixed_x = 1  # Fixed x-coordinate
center_y, center_z = 0, 0  # Center of the circle in y-z plane
num_steps = 200  # Number of steps to complete the circle
frequency = 50000  # Update frequency in Hz
# frequency = 10  # Update frequency in Hz
step = 0  # Current step
circle_period = 3
# with open(self.fifo_path, "w") as fifo:
t0 = time.time()
while True:
    angle = 2 * np.pi * (step / num_steps)
    angle = 2 * np.pi * (time.time() - t0) / circle_period
    step += 1
    if step >= num_steps:
        step = 0
    # Calculate y and z coordinates
    y = center_y + radius * np.cos(angle)
    z = center_z + radius * np.sin(angle)

    # Call the goto function with the constant x and calculated y, z coordinates
    goto(fixed_x, y, z, partid=2)
    goto(fixed_x, y, z, partid=1)
    time.sleep(1.0/frequency)
