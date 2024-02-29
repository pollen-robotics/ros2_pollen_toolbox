

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import time

class SinusoidalArmController(Node):
    def __init__(self):
        super().__init__('sinusoidal_arm_controller')
        # self.subscription = self.create_subscription(Float64, 'current_position_topic', self.listener_callback, 10)
        self.publisher = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.timer_period = 0.01  # seconds

        self.timer = self.create_timer(self.timer_period, self.timer_callback)


    def listener_callback(self, msg):
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        elapsed_time = current_time - self.start_time

        # Calculate sinusoidal command based on elapsed time

    def timer_callback(self):

        # def send(self, elapsed_time):
        current_time = time.time()
        f = 1.0
        command = math.sin(current_time * 2 * math.pi * f) * 0.05

        # Create and publish the command message
        command_msg = Float64MultiArray()
        command_msg.data = [command, -0.0,0.05,0.05, 0.05,0.05,0.05,0.0, 0.0, 0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        command_msg.data = [0.0, command,0.05,0.05, 0.05,0.05,0.05,0.0, 0.0, 0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        command_msg.data = [0.0, 0.0,command,0.05, 0.05,0.05,0.05,0.0, 0.0, 0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        command_msg.data = [0.0, 0.0,0.0,command, 0.05,0.05,0.05,0.0, 0.0, 0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        command_msg.data = [0.0, 0.0,0.0,0.0, command,0.05,0.05,0.0, 0.0, 0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        command_msg.data = [0.0, command,0.05,0.05, 0.05,0.05,0.05,0.0, 0.0, 0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        command_msg.data = [command, -0.0,0.05,0.05, 0.05,0.05,0.05,0.0, 0.0, 0.0,0.0,0.0,0.0,0.0,0.0,0.0]

        self.publisher.publish(command_msg)
        self.get_logger().info(f"Publishing command: {command_msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = SinusoidalArmController()
    # current_time = float(node.get_clock().now().seconds_nanoseconds()[0])
    current_time = time.time()
    f = 1.0

    # while True:
    # current_time = time.time()
    # node.send()
    # print(current_time)
    rclpy.spin(node)
    # rclpy.spin_once(node)
    # time.sleep(1/f)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




#!/usr/bin/python
# -*- coding: utf-8 -*-

#
#  File Name	: test_joints.py
#  Author	: Steve NGUYEN
#  Contact      : steve.nguyen.000@gmail.com
#  Created	: Monday, October 18 2021
#  Revised	:
#  Version	:
#  Target MCU	:
#
#  This code is distributed under the GNU Public License
# 		which can be found at http://www.gnu.org/licenses/gpl.txt
#
#
#  Notes:	notes
#

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from threading import Event

from copy import copy

from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import Quaternion, Point
import numpy as np
import time
import random


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(
            PoseStamped, '/target_pose', 10)
        self.timer_period = 0.01  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.i = 0
        self.logger = self.get_logger()
        self.t = 0.0

    def timer_callback(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        # msg.header.frame_id = 'left_tip'
        msg.header.frame_id = 'torso'

        s = 0.035*np.sin(2.0*np.pi*0.5*self.t)
        # s = self.t*0.001
        # # s = 0.0
        self.t += self.timer_period
        # for j in JOINTS_NAMES:
        #     msg.joint_names.append(j)
        #     p.positions.append(s)

        msg.pose.position.z = -0.3+s
        msg.pose.position.y = -0.16
        msg.pose.position.x = 0.37

        # msg.pose.position.z = 0.0+s
        # msg.pose.position.y = 0.16
        # msg.pose.position.x = 0.5

        # msg.pose.orientation.x = 0.0
        # msg.pose.orientation.y = 0.0
        # msg.pose.orientation.z = 0.0
        # msg.pose.orientation.w = 1.0

        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = -0.70710678
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 0.70710678

        self.logger.warn('msg: {}'.format(msg))

        self.publisher_.publish(msg)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()