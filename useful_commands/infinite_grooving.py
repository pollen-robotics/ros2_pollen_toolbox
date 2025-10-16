"""Example of how to draw a square with Reachy's right arm."""

import json
import logging
import signal
import sys
import threading
import time
import traceback

import numpy as np
import numpy.typing as npt
import rclpy
from control_msgs.msg import DynamicJointState
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from reachy2_sdk import ReachySDK
from reachy2_sdk.parts.joints_based_part import JointsBasedPart

# These are integer values between 0 and 100
TORQUE_LIMIT = 80
SPEED_LIMIT = 25

STOP = threading.Event()


class ReachySubscriber(Node):
    def __init__(self):
        super().__init__("reachy_subscriber")
        self.subscriber = self.create_subscription(
            DynamicJointState, "/dynamic_joint_states", self.update_state, 10
        )
        self.motor_temperatures = {}
        self.motor_currents = {}
        self.t0 = time.time()
        self.last_recordtime = self.t0
        self.initialized = False
        print("ReachySubscriber initialized")

    def update_state(self, msg):
        if time.time() - self.last_recordtime > 1:
            timestamp = f"{time.time() - self.t0:.3f}"
            for idx, name in enumerate(msg.joint_names):
                if "raw_motor" not in name:
                    continue

                if name not in self.motor_temperatures:
                    self.motor_temperatures[name] = {}
                if name not in self.motor_currents:
                    self.motor_currents[name] = {}

                kv = msg.interface_values[idx]
                data = {
                    iface: kv.values[i] for i, iface in enumerate(kv.interface_names)
                }

                if "motor_temperature" in data:
                    temp = data["motor_temperature"]
                elif "temperature" in data:
                    temp = data["temperature"]

                if "motor_currents" in data:
                    curr = data["motor_currents"]
                elif "currents" in data:
                    curr = data["currents"]

                self.motor_temperatures[name][timestamp] = float(temp)
                self.motor_currents[name][timestamp] = float(curr)
                self.last_recordtime = time.time()

    def print_motor_data(self):
        data = {
            "motor_temperatures": self.motor_temperatures,
            "motor_currents": self.motor_currents,
        }
        print(json.dumps(data))


def start_ros_background_spin(node):
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    t = threading.Thread(target=executor.spin, daemon=True)
    t.start()
    return executor, t


def stop_ros_background_spin(executor, node, thread):
    try:
        node.print_motor_data()
        executor.shutdown()
    except Exception:
        pass
    try:
        node.destroy_node()
    except Exception:
        pass
    try:
        rclpy.shutdown()
    except Exception:
        pass


def signal_handler(sig, frame):
    STOP.set()


def build_pose_matrix(x: float, y: float, z: float) -> npt.NDArray[np.float64]:
    """Build a 4x4 pose matrix for a given position in 3D space, with the effector at a fixed orientation.

    Args:
        x: The x-coordinate of the position.
        y: The y-coordinate of the position.
        z: The z-coordinate of the position.

    Returns:
        A 4x4 NumPy array representing the pose matrix.
    """
    # The effector is always at the same orientation in the world frame
    return np.array(
        [
            [0, 0, -1, x],
            [0, 1, 0, y],
            [1, 0, 0, z],
            [0, 0, 0, 1],
        ]
    )


def set_speed_and_torque_limits(reachy, torque_limit=100, speed_limit=25) -> None:
    """Set back speed and torque limits of all parts to given value."""
    if not reachy.info:
        reachy._logger.warning("Reachy is not connected!")
        return

    for part in reachy.info._enabled_parts.values():
        if issubclass(type(part), JointsBasedPart):
            part.set_speed_limits(speed_limit)
            part.set_torque_limits(torque_limit)
    time.sleep(0.5)


def draw_square(reachy: ReachySDK) -> None:
    """Draw a square path with Reachy's right arm in 3D space.

    This function commands Reachy's right arm to move in a square pattern
    using four predefined positions (A, B, C, and D) in the world frame.
    The square is drawn by moving the arm sequentially through these positions:
    - A: (0.4, -0.5, -0.2)
    - B: (0.4, -0.5, 0)
    - C: (0.4, -0.3, 0)
    - D: (0.4, -0.3, -0.2)

    see https://docs.pollen-robotics.com/sdk/first-moves/kinematics/ for Reachy's coordinate system

    Each movement uses inverse kinematics to calculate the required joint
    positions to achieve the target pose and then sends the commands to
    Reachy's arm to execute the movements.

    Args:
        reachy: An instance of the ReachySDK used to control the robot.
    """
    # Going from A to B
    r_target_pose = build_pose_matrix(0.4, -0.5, 0)
    l_target_pose = build_pose_matrix(0.4, 0.5, 0)
    r_ik = reachy.r_arm.inverse_kinematics(r_target_pose)
    l_ik = reachy.l_arm.inverse_kinematics(l_target_pose)
    reachy.r_arm.gripper.goto(50, percentage=True)
    reachy.l_arm.gripper.goto(50, percentage=True)
    reachy.head.goto([20, -10, 0])
    reachy.head.l_antenna.goto(20)
    reachy.head.r_antenna.goto(-20)
    reachy.r_arm.goto(r_ik, duration=2.0, degrees=True)
    reachy.l_arm.goto(l_ik, duration=2.0, degrees=True, wait=True)
    reachy.mobile_base.goto(
        x=0.0,
        y=0.0,
        theta=30.0,
        wait=False,
        degrees=True,
        distance_tolerance=0.05,
        angle_tolerance=5.0,
        timeout=10000,
    )

    # Going from B to C
    r_target_pose = build_pose_matrix(0.4, -0.3, 0)
    l_target_pose = build_pose_matrix(0.4, 0.3, 0)
    r_ik = reachy.r_arm.inverse_kinematics(r_target_pose)
    l_ik = reachy.l_arm.inverse_kinematics(l_target_pose)
    reachy.r_arm.gripper.goto(10, percentage=True)
    reachy.l_arm.gripper.goto(10, percentage=True)
    reachy.head.goto([-10, 30, 10])
    reachy.head.l_antenna.goto(50)
    reachy.head.r_antenna.goto(-50)
    reachy.r_arm.goto(r_ik, duration=2.0, degrees=True)
    reachy.l_arm.goto(l_ik, duration=2.0, degrees=True, wait=True)
    reachy.mobile_base.goto(
        x=0.0,
        y=0.0,
        theta=0.0,
        wait=False,
        degrees=True,
        distance_tolerance=0.05,
        angle_tolerance=5.0,
        timeout=10000,
    )

    # Going from C to D
    r_target_pose = build_pose_matrix(0.4, -0.3, -0.2)
    l_target_pose = build_pose_matrix(0.4, 0.3, -0.2)
    r_ik = reachy.r_arm.inverse_kinematics(r_target_pose)
    l_ik = reachy.l_arm.inverse_kinematics(l_target_pose)
    reachy.r_arm.gripper.goto(80, percentage=True)
    reachy.l_arm.gripper.goto(80, percentage=True)
    reachy.head.goto([0, 15, -10])
    reachy.head.l_antenna.goto(-20)
    reachy.head.r_antenna.goto(20)
    reachy.r_arm.goto(r_ik, duration=2.0, degrees=True)
    reachy.l_arm.goto(l_ik, duration=2.0, degrees=True, wait=True)
    reachy.mobile_base.goto(
        x=0.0,
        y=0.0,
        theta=-30.0,
        wait=False,
        degrees=True,
        distance_tolerance=0.05,
        angle_tolerance=5.0,
        timeout=10000,
    )

    # Going from D to A
    r_target_pose = build_pose_matrix(0.4, -0.5, -0.2)
    l_target_pose = build_pose_matrix(0.4, 0.5, -0.2)
    r_ik = reachy.r_arm.inverse_kinematics(r_target_pose)
    l_ik = reachy.l_arm.inverse_kinematics(l_target_pose)
    reachy.r_arm.gripper.goto(0, percentage=True)
    reachy.l_arm.gripper.goto(0, percentage=True)
    reachy.head.goto([15, 5, 20])
    reachy.head.l_antenna.goto(0)
    reachy.head.r_antenna.goto(0)
    reachy.r_arm.goto(r_ik, duration=2.0, degrees=True)
    reachy.l_arm.goto(l_ik, duration=2.0, degrees=True, wait=True)
    reachy.mobile_base.goto(
        x=0.0,
        y=0.0,
        theta=0.0,
        wait=False,
        degrees=True,
        distance_tolerance=0.05,
        angle_tolerance=5.0,
        timeout=10000,
    )


def goto_to_point_A(reachy: ReachySDK) -> None:
    """Move Reachy's right arm to Point A in 3D space.

    This function commands Reachy's right arm to move to a specified target position
    (Point A) in the world frame, which is located at (0.4, -0.5, -0.2).

    Args:
        reachy: An instance of the ReachySDK used to control the robot.
    """
    # position of point A in space
    r_target_pose = build_pose_matrix(0.4, -0.5, -0.2)
    l_target_pose = build_pose_matrix(0.4, 0.5, -0.2)
    # get the position in the joint space
    r_joints_positions = reachy.r_arm.inverse_kinematics(r_target_pose)
    l_joints_positions = reachy.l_arm.inverse_kinematics(l_target_pose)
    # move Reachy's right arm to this point
    reachy.r_arm.goto(r_joints_positions, duration=4, wait=False)
    reachy.l_arm.goto(l_joints_positions, duration=4, wait=True)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)

    starting_time = time.time()
    print("Reachy SDK example: draw square")

    logging.basicConfig(level=logging.INFO)
    reachy = ReachySDK(host="localhost")

    if not reachy.is_connected:
        exit("Reachy is not connected.")

    rclpy.init()
    reachy_subscriber = ReachySubscriber()
    executor, spin_thread = start_ros_background_spin(reachy_subscriber)

    print("Turning on Reachy")
    reachy.turn_on()
    reachy.mobile_base.reset_odometry()

    set_speed_and_torque_limits(
        reachy, torque_limit=TORQUE_LIMIT, speed_limit=SPEED_LIMIT
    )

    time.sleep(0.2)
    try:
        print("Move to point A, preparing infinite square drawing ...")
        reachy.r_arm.gripper.close()
        reachy.l_arm.gripper.close()
        goto_to_point_A(reachy)

        reachy.r_arm.gripper.open()
        reachy.l_arm.gripper.open()
        time.sleep(2.0)

        reachy.r_arm.gripper.close()
        reachy.l_arm.gripper.close()

        while (
            time.time() - starting_time < 7200 and not STOP.is_set()
        ):  # run for 2 hours
            print("Draw a square with the right arm ...")
            draw_square(reachy)
    except KeyboardInterrupt:
        print("Keyboard interrupt received, stopping...", file=sys.stderr)
    except Exception as e:
        print(f"An error occurred: {e}", file=sys.stderr)
        traceback.print_exc(file=sys.stderr)
    finally:
        reachy.cancel_all_goto()
        reachy.turn_off_smoothly()
        time.sleep(0.2)
        stop_ros_background_spin(executor, reachy_subscriber, spin_thread)
