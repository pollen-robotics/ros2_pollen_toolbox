import asyncio

from action_msgs.msg import GoalStatus
from trajectory_msgs.msg import JointTrajectoryPoint

from pollen_msgs.action import GotoTrajectory


import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from typing import List
import time


class GotoActionClient(Node):
    def __init__(self):
        super().__init__("goto_action_client")
        self.prefixes = ["r_arm", "l_arm", "neck"]
        self.goto_action_client = {}
        for prefix in self.prefixes:
            self.goto_action_client[prefix] = ActionClient(
                self, GotoTrajectory, f"{prefix}_goto"
            )
            self.get_logger().info(f"Waiting for action server {prefix}_goto...")
            self.goto_action_client[prefix].wait_for_server()

    def feedback_callback(self, feedback):
        self.get_logger().info("FEEDBAAAAACK")
        self.get_logger().info(
            f"Received feedback. status: {feedback.feedback.feedback.status}, time to completion: {feedback.feedback.feedback.time_to_completion}"
        )

    async def send_goal(
        self, part: str, joints: List[str], goal_positions: List[float], duration: float
    ):
        goal_msg = GotoTrajectory.Goal()

        goal_msg.trajectory.joint_names = joints
        p = JointTrajectoryPoint()
        p.positions = goal_positions
        p.time_from_start.sec = int(duration)
        p.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        goal_msg.trajectory.points = [p]

        self.get_logger().info("Sending goal request...")

        # configuring a feedback callback apparently deadlocks the client??
        goal_handle = await self.goto_action_client[part].send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self.get_logger().info("feedback_callback setuped")

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            return

        self.get_logger().info("Goal accepted")
        res = await goal_handle.get_result_async()

        result = res.result
        status = res.status
        self.get_logger().info("Goal async finished")

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal succeeded! Result: {0}".format(result))
        else:
            self.get_logger().info("Goal failed with status: {0}".format(status))
        return result, status


async def spinning(node):
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)
        await asyncio.sleep(0.001)


async def blocking_demo(action_client):
    logger = rclpy.logging.get_logger("goto_action_client")

    logger.info(f"$$$$$$ EXAMPLE 1: blocking calls")
    result, status = await action_client.send_goal(
        "r_arm", ["r_shoulder_pitch"], [-1.0], 2.0
    )
    logger.info(f"A) result {result} and status flag {status}")

    result, status = await action_client.send_goal(
        "l_arm", ["l_shoulder_pitch"], [-1.0], 2.0
    )
    logger.info(f"B) result {result} and status flag {status}")

    result, status = await action_client.send_goal("neck", ["neck_roll"], [0.2], 2.0)
    logger.info(f"C) result {result} and status flag {status}")

    logger.info(f"$$$$$$ EXAMPLE 1: going back to start position")
    result, status = await action_client.send_goal(
        "r_arm", ["r_shoulder_pitch"], [0.0], 2.0
    )
    logger.info(f"A) result {result} and status flag {status}")

    result, status = await action_client.send_goal(
        "l_arm", ["l_shoulder_pitch"], [0.0], 2.0
    )
    logger.info(f"B) result {result} and status flag {status}")

    result, status = await action_client.send_goal("neck", ["neck_roll"], [0.0], 2.0)
    logger.info(f"C) result {result} and status flag {status}")


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
        logger.info("result {} and status flag {}".format(*task.result()))
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
        logger.info("result {} and status flag {}".format(*task.result()))


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
        logger.info("result {} and status flag {}".format(*task.result()))

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
        logger.info("result {} and status flag {}".format(*task.result()))


async def run_demo(args, loop):
    logger = rclpy.logging.get_logger("goto_action_client")

    # init ros2
    rclpy.init(args=args)

    # create node
    action_client = GotoActionClient()

    # start spinning
    spin_task = loop.create_task(spinning(action_client))

    # Demo 1: blocking calls
    await blocking_demo(action_client)

    # Demo 2: non-blocking calls called simultaneously
    await non_blocking_demo(action_client, loop)

    # Demo 3: non-blocking calls called with a delay
    await non_blocking_demo_delay(action_client, loop)

    # cancel spinning task
    spin_task.cancel()
    try:
        await spin_task
    except asyncio.exceptions.CancelledError:
        pass
    rclpy.shutdown()


def main(args=None):
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run_demo(args, loop=loop))


if __name__ == "__main__":
    main()
