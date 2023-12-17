import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import random
import time
from control_msgs.msg import DynamicJointState, InterfaceValue
import time
from pollen_msgs.action import Goto
from control_msgs.msg import DynamicJointState, InterfaceValue

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from threading import Event
import threading
import numpy as np
import collections
import pickle


class RandomCommandPublisher(Node):
    def __init__(self):
        super().__init__("random_command_publisher")
        self.joint_commands_pub = self.create_publisher(
            msg_type=DynamicJointState, topic="/dynamic_joint_commands", qos_profile=5
        )
        self.timer = self.create_timer(1.0 / 150.0, self.publish_random_command)

    def publish_random_command(self):
        joint_names = [
            "r_shoulder_pitch",
            "r_shoulder_roll",
            "r_elbow_yaw",
            "r_elbow_pitch",
            "r_wrist_roll",
            "r_wrist_pitch",
            "r_wrist_yaw",
        ]

        # Generate random positions for each joint
        points = [random.uniform(-1.0, 1.0) for _ in joint_names]

        cmd_msg = DynamicJointState()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        # cmd_msg.joint_names=joints

        for j, p in zip(joint_names, points):
            cmd_msg.joint_names.append(j)
            inter = InterfaceValue()
            inter.interface_names.append("position")
            inter.values.append(p)
            cmd_msg.interface_values.append(inter)

        self.joint_commands_pub.publish(cmd_msg)


class CentralJointStateHandler(Node):
    def __init__(self):
        super().__init__("central_joint_state_handler")
        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.on_joint_state, 5
        )
        self.joint_state = {}
        # read from pickle file
        # self.joint_state = pickle.load(open("/home/reachy/joint_state.pkl", "rb"))
        self.joint_state_ready = Event()
        # self.joint_state_ready.set()

    def on_joint_state(self, state: JointState):
        """Retreive the joint state from /joint_states."""
        if not self.joint_state_ready.is_set():
            for uid, name in enumerate(state.name):
                self.joint_state[name] = {}
            self.joint_state_ready.set()

        for name, pos, vel, effort in zip(
            state.name, state.position, state.velocity, state.effort
        ):
            self.joint_state[name]["position"] = pos
            self.joint_state[name]["velocity"] = vel
            self.joint_state[name]["effort"] = effort
        # save joint state in a pickle file
        # with open("/home/reachy/joint_state.pkl", "wb") as f:
        #     pickle.dump(self.joint_state, f)


def main(args=None):
    # rclpy.init(args=args)
    # random_command_publisher = RandomCommandPublisher()
    # rclpy.spin(random_command_publisher)
    # random_command_publisher.destroy_node()
    # rclpy.shutdown()

    rclpy.init(args=args)
    random_command_publisher = RandomCommandPublisher()

    mult_executor = MultiThreadedExecutor()
    joint_state_handler = CentralJointStateHandler()

    mult_executor.add_node(random_command_publisher)
    mult_executor.add_node(joint_state_handler)
    executor_thread = threading.Thread(target=mult_executor.spin, daemon=True)
    executor_thread.start()
    rate = random_command_publisher.create_rate(2.0)

    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
    executor_thread.join()


if __name__ == "__main__":
    main()
