import asyncio
import copy
from typing import List

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from pollen_msgs.action import Goto
from rclpy.action import ActionClient
from rclpy.node import Node
from reachy_sdk_server.conversion import matrix_to_pose, pose_to_matrix
from sensor_msgs.msg import JointState


class GotoActionClient(Node):
    def __init__(self):
        super().__init__("goto_action_client")
        self.prefixes = ["r_arm"]
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
        request.sampling_freq = 150.0
        request.safety_on = False

        request.goal_joints = JointState()
        request.goal_joints.name = joint_names
        request.goal_joints.position = goal_positions
        request.goal_joints.velocity = goal_velocities
        request.goal_joints.effort = []  # Not implemented for now

        self.get_logger().info("Sending goal request...")

        goal_handle = await self.goto_action_client[part].send_goal_async(goal_msg, feedback_callback=feedback_callback)
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


async def run_demo(args, loop):
    logger = rclpy.logging.get_logger("goto_action_client")

    # init ros2
    rclpy.init(args=args)

    # create node
    action_client = GotoActionClient()

    # start spinning
    spin_task = loop.create_task(spinning(action_client))

    while True:
        await square_demo(action_client, loop)


def main(args=None):
    loop = asyncio.get_event_loop()
    done, _pending = loop.run_until_complete(run_demo(args, loop=loop))

    for task in done:
        task.result()  # raises exceptions if any


if __name__ == "__main__":
    main()
