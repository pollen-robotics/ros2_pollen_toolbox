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
        self._action_client = ActionClient(self, GotoTrajectory, "right_arm_goto")
        self.get_logger().info("Waiting for action server...")
        self._action_client.wait_for_server()

    def feedback_callback(self, feedback):
        self.get_logger().info("FEEDBAAAAACK")
        self.get_logger().info(
            f"Received feedback. status: {feedback.feedback.feedback.status}, time to completion: {feedback.feedback.feedback.time_to_completion}"
        )

    async def send_goal(
        self, joints: List[str], goal_positions: List[float], duration: float
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
        goal_handle = await self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self.get_logger().info("feedback_callback setuped")

        # goal_handle = await self._action_client.send_goal_async(goal_msg)

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            # TODO eventually do something here to handle goal rejection
            return

        self.get_logger().info("Goal accepted")

        # seems to deadlock here
        res = await goal_handle.get_result_async()

        result = res.result
        status = res.status
        self.get_logger().info("Goal async finished")

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal succeeded! Result: {0}".format(result))
        else:
            self.get_logger().info("Goal failed with status: {0}".format(status))
        return result, status

    # def get_result_callback(self, future):
    #     result = future.result().result
    #     self.get_logger().warn(f'Result: {result}')
    #     self.status = GoalStatus.STATUS_SUCCEEDED


async def spinning(node):
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)
        await asyncio.sleep(0.001)


async def run(args, loop):
    logger = rclpy.logging.get_logger("minimal_action_client")

    # init ros2
    rclpy.init(args=args)

    # create node
    action_client = GotoActionClient()

    # start spinning
    spin_task = loop.create_task(spinning(action_client))

    # # Parallel example
    # # execute goal request and schedule in loop
    # logger.info(f"Creating task1")
    # my_task1 = loop.create_task(
    #     action_client.send_goal(["r_shoulder_pitch"], [-1.0], 2.0)
    # )
    # time.sleep(0.05)
    # logger.info(f"Creating task2")
    # my_task2 = loop.create_task(
    #     action_client.send_goal(["r_shoulder_pitch"], [0.0], 2.0)
    # )
    # logger.info(f"Gluing tasks and waiting")

    # TODO Rémi debug this :
    # logger.info(f'Creating task1')
    # my_task1 = loop.create_task(action_client.send_goal(['r_shoulder_pitch', 'r_elbow_pitch'], [-1.0, -1.0], 2.0))
    # logger.info(f'Creating task2')
    # my_task2 = loop.create_task(action_client.send_goal(['r_shoulder_pitch', 'r_elbow_pitch'], [0.0, 0.0], 2.0))

    # glue futures together and wait
    # wait_future = asyncio.wait([my_task1, my_task2])
    # run event loop

    # finished, unfinished = await wait_future
    # logger.info(f"unfinished: {len(unfinished)}")
    # for task in finished:
    #     logger.info("result {} and status flag {}".format(*task.result()))

    # Sequence
    result, status = await loop.create_task(
        action_client.send_goal(["r_shoulder_pitch"], [-1.0], 2.0)
    )
    logger.info(f"A) result {result} and status flag {status}")
    result, status = await loop.create_task(
        action_client.send_goal(["r_shoulder_pitch"], [0.0], 2.0)
    )
    logger.info(f"B) result {result} and status flag {status}")

    # cancel spinning task
    spin_task.cancel()
    try:
        await spin_task
    except asyncio.exceptions.CancelledError:
        pass
    rclpy.shutdown()


def main(args=None):
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run(args, loop=loop))


if __name__ == "__main__":
    main()
