import copy
import rclpy
import asyncio
import numpy as np

from action_msgs.msg import GoalStatus

from pollen_msgs.action import Goto
from sensor_msgs.msg import JointState


from rclpy.action import ActionClient
from rclpy.node import Node
from typing import List

from reachy_sdk_server.conversion import matrix_to_pose, pose_to_matrix


class GotoActionClient(Node):
    def __init__(self):
        super().__init__("goto_action_client")
        self.prefixes = ["r_arm", "l_arm", "neck"]
        # self.prefixes = ["r_arm"]
        self.goto_action_client = {}
        for prefix in self.prefixes:
            self.goto_action_client[prefix] = ActionClient(self, Goto, f"{prefix}_goto")
            self.get_logger().info(f"Waiting for action server {prefix}_goto...")
            self.goto_action_client[prefix].wait_for_server()

    def feedback_callback_default(self, feedback):
        self.get_logger().info(
            f"Received feedback. status: {feedback.feedback.feedback.status}, time to completion: {feedback.feedback.feedback.time_to_completion}, commands_sent {feedback.feedback.feedback.commands_sent}"
        )

    async def send_goal(
        self,
        part: str,
        joint_names: List[str],
        goal_positions: List[float],
        duration: float,
        goal_velocities: List[float] = [],
        mode: str = "minimum_jerk",  # "linear" or "minimum_jerk"
        feedback_callback=None,
        return_handle=False,
    ):
        goal_msg = Goto.Goal()
        request = goal_msg.request  # This is of type pollen_msgs/GotoRequest

        request.duration = duration
        request.mode = mode
        request.sampling_freq = 200.0
        request.safety_on = False

        request.goal_joints = JointState()
        request.goal_joints.name = joint_names
        request.goal_joints.position = goal_positions
        request.goal_joints.velocity = goal_velocities
        request.goal_joints.effort = []  # Not implemented for now

        self.get_logger().info("Sending goal request...")

        goal_handle = await self.goto_action_client[part].send_goal_async(
            goal_msg, feedback_callback=feedback_callback
        )
        self.get_logger().info("feedback_callback setuped")

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            return

        self.get_logger().info("Goal accepted")

        if return_handle:
            return goal_handle
        else:
            res = await goal_handle.get_result_async()
            result = res.result
            status = res.status
            self.get_logger().info(f"Goto finished. Result: {result.result.status}")
            return result, status


async def spinning(node):
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)
        await asyncio.sleep(0.001)


async def blocking_demo(action_client):
    logger = rclpy.logging.get_logger("goto_action_client")

    logger.info(f"$$$$$$ EXAMPLE 1: blocking calls")
    # Setting the feedback callback only once because it's very verbose
    result, status = await action_client.send_goal(
        "r_arm",
        ["r_shoulder_pitch"],
        [-1.0],
        2.0,
        feedback_callback=action_client.feedback_callback_default,
    )

    # An example on how to read result and status:
    if status == GoalStatus.STATUS_SUCCEEDED:
        logger.info(f"Goal succeeded! Result: {result.result.status}")
    else:
        logger.info(f"Goal failed. Result: {result.result.status}")

    result, status = await action_client.send_goal(
        "l_arm",
        ["l_shoulder_pitch"],
        [-1.0],
        2.0,
    )

    result, status = await action_client.send_goal("neck", ["neck_roll"], [0.2], 2.0)

    logger.info(f"$$$$$$ EXAMPLE 1: going back to start position")
    result, status = await action_client.send_goal(
        "r_arm",
        ["r_shoulder_pitch"],
        [0.0],
        2.0,
    )

    result, status = await action_client.send_goal(
        "l_arm",
        ["l_shoulder_pitch"],
        [0.0],
        2.0,
    )

    result, status = await action_client.send_goal(
        "neck",
        ["neck_roll"],
        [0.0],
        2.0,
    )


async def non_blocking_demo(action_client, loop):
    logger = rclpy.logging.get_logger("goto_action_client")

    logger.info(f"$$$$$$ EXAMPLE 2: simultaneous async calls")
    my_task1 = loop.create_task(
        action_client.send_goal("r_arm", ["r_shoulder_pitch"], [-1.0], 2.0)
    )
    my_task2 = loop.create_task(
        action_client.send_goal("l_arm", ["l_shoulder_pitch"], [-1.0], 2.0)
    )
    my_task3 = loop.create_task(
        action_client.send_goal("neck", ["neck_roll"], [0.2], 2.0)
    )
    logger.info(f"Gluing tasks and waiting")

    wait_future = asyncio.wait([my_task1, my_task2, my_task3])
    # run event loop
    finished, unfinished = await wait_future
    logger.info(f"unfinished: {len(unfinished)}")
    for task in finished:
        result, status = task.result()
        logger.info(f"Result: {result.result.status}")

    logger.info(f"$$$$$$ EXAMPLE 2: Going back to start position")
    my_task1 = loop.create_task(
        action_client.send_goal("r_arm", ["r_shoulder_pitch"], [0.0], 2.0)
    )
    my_task2 = loop.create_task(
        action_client.send_goal("l_arm", ["l_shoulder_pitch"], [0.0], 2.0)
    )
    my_task3 = loop.create_task(
        action_client.send_goal("neck", ["neck_roll"], [0.0], 2.0)
    )
    logger.info(f"Gluing tasks and waiting")
    wait_future = asyncio.wait([my_task1, my_task2, my_task3])
    # run event loop
    finished, unfinished = await wait_future
    logger.info(f"unfinished: {len(unfinished)}")
    for task in finished:
        result, status = task.result()
        logger.info(f"Result: {result.result.status}")


async def non_blocking_demo_delay(action_client, loop):
    logger = rclpy.logging.get_logger("goto_action_client")
    logger.info(f"$$$$$$ EXAMPLE 3: delayed async calls")
    # execute goal request and schedule in loop
    logger.info(f"Creating task1")
    my_task1 = loop.create_task(
        action_client.send_goal("r_arm", ["r_shoulder_pitch"], [-1.0], 2.0)
    )
    await asyncio.sleep(0.5)
    logger.info(f"Creating task2")
    my_task2 = loop.create_task(
        action_client.send_goal("l_arm", ["l_shoulder_pitch"], [-1.0], 2.0)
    )
    await asyncio.sleep(0.5)
    logger.info(f"Creating task3")
    my_task3 = loop.create_task(
        action_client.send_goal("neck", ["neck_roll"], [0.2], 2.0)
    )
    logger.info(f"Gluing tasks and waiting")
    wait_future = asyncio.wait([my_task1, my_task2, my_task3])
    # run event loop
    finished, unfinished = await wait_future
    logger.info(f"unfinished: {len(unfinished)}")
    for task in finished:
        result, status = task.result()
        logger.info(f"Result: {result.result.status}")

    logger.info(f"$$$$$$ EXAMPLE 3: Going back to start position")
    my_task1 = loop.create_task(
        action_client.send_goal("r_arm", ["r_shoulder_pitch"], [0.0], 2.0)
    )
    await asyncio.sleep(0.5)
    my_task2 = loop.create_task(
        action_client.send_goal("l_arm", ["l_shoulder_pitch"], [0.0], 2.0)
    )
    await asyncio.sleep(0.5)
    logger.info(f"Creating task3")
    my_task3 = loop.create_task(
        action_client.send_goal("neck", ["neck_roll"], [0.0], 2.0)
    )
    logger.info(f"Gluing tasks and waiting")
    wait_future = asyncio.wait([my_task1, my_task2, my_task3])
    # run event loop
    finished, unfinished = await wait_future
    logger.info(f"unfinished: {len(unfinished)}")
    for task in finished:
        result, status = task.result()
        logger.info(f"Result: {result.result.status}")


async def square_demo(action_client, loop):
    logger = rclpy.logging.get_logger("goto_action_client")
    logger.info(f"$$$$$$ EXAMPLE 4: complete IK calls")

    r_square = [
        [7.53, -22.16, -30.57, -69.92, 14.82, 20.59, 18.28],
        [-6.43, -34.65, -46.71, -109.05, 42.49, -28.29, 45.83],
        [-22.53, 5.92, 2.71, -123.43, -31.76, -50.2, -30.13],
        [2.66, 6.08, -1.9, -83.19, -12.97, 10.76, -3.67],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    ]

    joint_names = [
        "r_shoulder_pitch",
        "r_shoulder_roll",
        "r_elbow_yaw",
        "r_elbow_pitch",
        "r_wrist_roll",
        "r_wrist_pitch",
        "r_wrist_yaw",
    ]

    for p in r_square:
        # Setting values of p to radians
        p = [np.deg2rad(i) for i in p]
        result, status = await action_client.send_goal("r_arm", joint_names, p, 0.5)
        logger.info(f"Result: {result.result.status}")
        # await asyncio.sleep(0.2)

    # # Cancel version
    # for p in r_square:
    #     # Setting values of p to radians
    #     p = [np.deg2rad(i) for i in p]
    #     goal_handle = await action_client.send_goal(
    #         "r_arm",
    #         joint_names,
    #         p,
    #         0.5,
    #         return_handle=True,
    #         feedback_callback=action_client.feedback_callback_default,
    #     )
    #     await asyncio.sleep(0.4)
    #     await goal_handle.cancel_goal_async()


async def cancel_demo(action_client, loop):
    logger = rclpy.logging.get_logger("goto_action_client")
    logger.info(f"$$$$$$ EXAMPLE 5: cancel demo")

    p = [7.53, -22.16, -30.57, -69.92, 14.82, 20.59, 18.28]

    joint_names = [
        "r_shoulder_pitch",
        "r_shoulder_roll",
        "r_elbow_yaw",
        "r_elbow_pitch",
        "r_wrist_roll",
        "r_wrist_pitch",
        "r_wrist_yaw",
    ]

    # Setting values of p to radians
    p = [np.deg2rad(i) for i in p]
    goal_handle = await action_client.send_goal(
        "r_arm",
        joint_names,
        p,
        2.0,
        return_handle=True,
        feedback_callback=action_client.feedback_callback_default,
    )
    await asyncio.sleep(1.0)
    await goal_handle.cancel_goal_async()
    logger.info(f"Goal canceled!")

    logger.info(f"Going back to 0")

    p = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    await action_client.send_goal("r_arm", joint_names, p, 2.0)


async def continuous_speed_demo(action_client, loop):
    logger = rclpy.logging.get_logger("goto_action_client")
    logger.info(f"$$$$$$ EXAMPLE 6: continuous speed demo")
    p = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    joint_names = [
        "r_shoulder_pitch",
        "r_shoulder_roll",
        "r_elbow_yaw",
        "r_elbow_pitch",
        "r_wrist_roll",
        "r_wrist_pitch",
        "r_wrist_yaw",
    ]
    logger.info(
        f"$$$In this first case, a continuation movement is scheduled before the first goto ended. It will be played as soon as the first one is finished"
    )
    logger.info(f"Creating task1")
    my_task1 = loop.create_task(
        action_client.send_goal("r_arm", ["r_shoulder_pitch"], [-0.5], 1.0)
    )
    await asyncio.sleep(0.5)
    logger.info(f"Creating task2")
    my_task2 = loop.create_task(
        action_client.send_goal("r_arm", ["r_shoulder_pitch"], [-1.0], 1.0)
    )

    logger.info(f"Gluing tasks and waiting")
    wait_future = asyncio.wait([my_task1, my_task2])
    # run event loop
    finished, unfinished = await wait_future
    logger.info(f"unfinished: {len(unfinished)}")
    for task in finished:
        result, status = task.result()
        logger.info(f"Result: {result.result.status}")

    logger.info(f"Going back to start position")
    await action_client.send_goal("r_arm", joint_names, p, 2.0)

    logger.info(
        f"$$$In this second case, a continuation movement will also be scheduled before the first goto ended. However, this time the mode is set to linear so the movement appears to be continuous"
    )
    logger.info(f"Creating task1")
    my_task1 = loop.create_task(
        action_client.send_goal(
            "r_arm",
            ["r_shoulder_pitch"],
            [-0.5],
            1.0,
            mode="linear",
        )
    )
    await asyncio.sleep(0.5)
    logger.info(f"Creating task2")
    my_task2 = loop.create_task(
        action_client.send_goal(
            "r_arm", ["r_shoulder_pitch"], [-1.0], 1.0, mode="linear"
        )
    )

    logger.info(f"Gluing tasks and waiting")
    wait_future = asyncio.wait([my_task1, my_task2])
    # run event loop
    finished, unfinished = await wait_future
    logger.info(f"unfinished: {len(unfinished)}")
    for task in finished:
        result, status = task.result()
        logger.info(f"Result: {result.result.status}")

    logger.info(f"Going back to start position")
    await action_client.send_goal("r_arm", joint_names, p, 2.0)

    # Note: One could set a starting speed to create continuous movements with the minimum jerk mode. But the measured speed of the joints (that will be used as starting state) have to be accurate. Otherwise, the movement will not be continuous.


async def all_at_once_demo(action_client, loop):
    logger = rclpy.logging.get_logger("goto_action_client")
    logger.info(f"$$$$$$ EXAMPLE 4: complete IK calls")

    r_square = [
        [7.53, -22.16, -30.57, -69.92, 14.82, 20.59, 18.28],
        [-6.43, -34.65, -46.71, -109.05, 42.49, -28.29, 45.83],
        [-22.53, 5.92, 2.71, -123.43, -31.76, -50.2, -30.13],
        [2.66, 6.08, -1.9, -83.19, -12.97, 10.76, -3.67],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    ]
    neck_angles = [
        [0.2, 0.2, 0.2],
        [0.0, 0.0, 0.0],
        [-0.2, -0.2, -0.2],
        [0.0, 0.0, 0.0],
    ]

    r_joint_names = [
        "r_shoulder_pitch",
        "r_shoulder_roll",
        "r_elbow_yaw",
        "r_elbow_pitch",
        "r_wrist_roll",
        "r_wrist_pitch",
        "r_wrist_yaw",
    ]

    l_joint_names = [
        "l_shoulder_pitch",
        "l_shoulder_roll",
        "l_elbow_yaw",
        "l_elbow_pitch",
        "l_wrist_roll",
        "l_wrist_pitch",
        "l_wrist_yaw",
    ]

    for index, p in enumerate(r_square):
        # Setting values of p to radians
        p_r = [np.deg2rad(i) for i in p]
        # p_l is like p_r but inverted when index==1
        p_l = copy.deepcopy(p_r)
        p_l[1] = -p_l[1]
        p_l[2] = -p_l[2]
        p_l[4] = -p_l[4]
        p_l[6] = -p_l[6]

        my_task1 = loop.create_task(
            action_client.send_goal("r_arm", r_joint_names, p_r, 0.5)
        )
        my_task2 = loop.create_task(
            action_client.send_goal("l_arm", l_joint_names, p_l, 0.5)
        )
        my_task3 = loop.create_task(
            action_client.send_goal(
                "neck",
                ["neck_roll", "neck_pitch", "neck_yaw"],
                neck_angles[index % len(neck_angles)],
                0.5,
            )
        )

        wait_future = asyncio.wait([my_task1, my_task2, my_task3])
        finished, unfinished = await wait_future


async def run_demo(args, loop):
    logger = rclpy.logging.get_logger("goto_action_client")

    # init ros2
    rclpy.init(args=args)

    # create node
    action_client = GotoActionClient()

    # start spinning
    spin_task = loop.create_task(spinning(action_client))

    # # Demo 1: blocking calls
    await blocking_demo(action_client)

    # Demo 2: non-blocking calls called simultaneously
    await non_blocking_demo(action_client, loop)

    # Demo 3: non-blocking calls called with a delay
    await non_blocking_demo_delay(action_client, loop)

    # Demo 4: square
    await square_demo(action_client, loop)

    # Demo 5: cancel
    await cancel_demo(action_client, loop)

    # Demo 6: continuous speed
    await continuous_speed_demo(action_client, loop)

    # Demo 7: all at once
    while True:
        await all_at_once_demo(action_client, loop)

    # cancel spinning task
    spin_task.cancel()
    try:
        await spin_task
    except asyncio.exceptions.CancelledError:
        pass
    rclpy.shutdown()


def main(args=None):
    loop = asyncio.get_event_loop()
    done, _pending = loop.run_until_complete(run_demo(args, loop=loop))

    for task in done:
        task.result()  # raises exceptions if any


if __name__ == "__main__":
    main()
