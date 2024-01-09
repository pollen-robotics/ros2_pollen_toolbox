from calendar import c
import time
from pollen_msgs.action import Goto
from control_msgs.msg import DynamicJointState, InterfaceValue

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from .interpolation import InterpolationMode

from threading import Event
import threading
import numpy as np
import collections


class GotoActionServer(Node):
    def __init__(self, name_prefix):
        super().__init__(f"{name_prefix}_goto_action_server")
        self._goal_queue = collections.deque()
        self._goal_queue_lock = threading.Lock()
        self._current_goal = None

        self.joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.on_joint_state,
            5,
        )
        self.dynamic_joint_commands_pub = self.create_publisher(
            msg_type=DynamicJointState,
            topic="/dynamic_joint_commands",
            qos_profile=5,
        )
        self.joint_state = {}
        self.joint_state_ready = Event()

        while not self.joint_state_ready.is_set():
            # spin once
            rclpy.spin_once(self, timeout_sec=0.1)
            self.get_logger().warn("Waiting for joint_state_ready to be set")
            time.sleep(0.1)

        self.dynamic_joint_commands = self.init_dynamic_joint_commands()
        self.timer = self.create_timer(1.0 / 150.0, self.publish_dynamic_joint_commands)

        self._action_server = ActionServer(
            self,
            Goto,
            f"{name_prefix}_goto",
            # handle_accepted_callback=self.handle_accepted_callback,
            execute_callback=self.execute_callback,
        )

        # Not sending the feedback every tick
        self.nb_commands_per_feedback = 10
        self.get_logger().info("Goto action server init.")

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

    def init_dynamic_joint_commands(self):
        cmd_msg = DynamicJointState()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()

        for name in self.joint_state.keys():
            cmd_msg.joint_names.append(name)
            inter = InterfaceValue()
            inter.interface_names.append("position")
            inter.values.append(self.joint_state[name]["position"])
            cmd_msg.interface_values.append(inter)

        return cmd_msg

    def publish_dynamic_joint_commands(self):
        self.dynamic_joint_commands.header.stamp = self.get_clock().now().to_msg()
        self.dynamic_joint_commands_pub.publish(self.dynamic_joint_commands)

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def compute_traj(
        self,
        duration,
        starting_pos,
        goal_pos,
        starting_vel=None,
        goal_vel=None,
        starting_acc=None,
        goal_acc=None,
        interpolation_mode: InterpolationMode = InterpolationMode.MINIMUM_JERK,
    ):
        return interpolation_mode(
            starting_pos,
            goal_pos,
            duration,
            starting_vel,
            starting_acc,
            goal_vel,
            goal_acc,
        )

    def prepare_data(self, goto_request):
        # goto_request is of type pollen_msgs/GotoRequest
        goal_pos_dict = {}
        goal_vel_dict = {}
        goal_acc_dict = {}

        start_pos_dict = {}
        start_vel_dict = {}
        start_acc_dict = {}

        for it, joint_name in enumerate(goto_request.goal_joints.name):
            goal_pos_dict[joint_name] = goto_request.goal_joints.position[it]

            start_pos_dict[joint_name] = self.joint_state[joint_name]["position"]

            if len(goto_request.goal_joints.velocity) > 0:
                goal_vel_dict[joint_name] = goto_request.goal_joints.velocity[it]
                start_vel_dict[joint_name] = self.joint_state[joint_name]["velocity"]

            else:
                goal_vel_dict[joint_name] = 0.0
                start_vel_dict[joint_name] = 0.0

            # Acceleration is not implemented for now
            goal_acc_dict[joint_name] = 0.0
            start_acc_dict[joint_name] = 0.0

        return (
            start_pos_dict,
            goal_pos_dict,
            start_vel_dict,
            goal_vel_dict,
            start_acc_dict,
            goal_acc_dict,
        )

    def cmd_pub(self, joints, points):
        for name, p in zip(joints, points):
            # Find index of name in self.dynamic_joint_commands.joint_names
            idx = self.dynamic_joint_commands.joint_names.index(name)
            if idx == -1:
                self.get_logger().error(f"Joint {name} not found in joint_names")
                return
            # print self.dynamic_joint_commands
            self.dynamic_joint_commands.interface_values[idx].values[0] = p

    def goto(
        self, traj_func, joints, duration, goal_handle, sampling_freq: float = 100
    ):
        length = round(duration * sampling_freq)
        if length < 1:
            raise ValueError(
                f"Goto length too short! (incoherent duration {duration} or sampling_freq {sampling_freq})!"
            )

        t0 = self.get_clock().now()

        rate = self.create_rate(sampling_freq)
        commands_sent = 0
        while True:
            elapsed_time = self.get_clock().now() - t0
            elapsed_time = elapsed_time.nanoseconds * 1e-9

            if elapsed_time > duration:
                self.get_logger().info(f"goto finished")
                break

            point = traj_func(elapsed_time)
            self.cmd_pub(joints, point)

            commands_sent += 1

            rate.sleep()

        return "finished"

    def goto_time(
        self, traj_func, joints, duration, goal_handle, sampling_freq: float = 100
    ):
        length = round(duration * sampling_freq)
        if length < 1:
            raise ValueError(
                f"Goto length too short! (incoherent duration {duration} or sampling_freq {sampling_freq})!"
            )

        t0 = time.time()

        # rate = self.create_rate(sampling_freq)
        commands_sent = 0
        while True:
            elapsed_time = time.time() - t0

            if elapsed_time > duration:
                self.get_logger().info(f"goto finished")
                break

            point = traj_func(elapsed_time)
            self.cmd_pub(joints, point)
            commands_sent += 1

            time.sleep(1.0 / sampling_freq)

        return "finished"

    def execute_callback(self, goal_handle):
        """Execute a goal."""

        start_time = time.time()
        self.get_logger().info(f"Executing goal...")
        ret = ""
        self.get_logger().warn(
            f"Timestamp: {1000*(time.time() - start_time):.2f}ms after joint_state_ready.is_set()"
        )
        goto_request = goal_handle.request.request  # pollen_msgs/GotoRequest
        duration = goto_request.duration
        mode = goto_request.mode
        sampling_freq = goto_request.sampling_freq
        safety_on = goto_request.safety_on

        (
            start_pos_dict,
            goal_pos_dict,
            start_vel_dict,
            goal_vel_dict,
            start_acc_dict,
            goal_acc_dict,
        ) = self.prepare_data(goto_request)
        self.get_logger().warn(
            f"Timestamp: {1000*(time.time() - start_time):.2f}ms after prepare data"
        )
        self.get_logger().debug(
            f"start_pos: {start_pos_dict} goal_pos: {goal_pos_dict} duration: {duration} start_vel: {start_vel_dict} start_acc: {start_acc_dict} goal_vel: {goal_vel_dict} goal_acc: {goal_acc_dict}"
        )

        traj_func = self.compute_traj(
            duration,
            np.array(list(start_pos_dict.values())),
            np.array(list(goal_pos_dict.values())),
            np.array(list(start_vel_dict.values())),
            np.array(list(goal_vel_dict.values())),
            np.array(list(start_acc_dict.values())),
            np.array(list(goal_acc_dict.values())),
            interpolation_mode=InterpolationMode.MINIMUM_JERK,
        )
        self.get_logger().warn(
            f"Timestamp: {1000*(time.time() - start_time):.2f}ms after compute_traj"
        )
        joints = start_pos_dict.keys()
        ret = self.goto(  # "goto_time" works, "goto" has the slow frequency issue
            traj_func,
            joints,
            duration,
            goal_handle,
            sampling_freq=sampling_freq,
        )
        self.get_logger().warn(
            f"Timestamp: {1000*(time.time() - start_time):.2f}ms after goto"
        )

        if ret == "finished":
            goal_handle.succeed()

        # Populate result message
        result = Goto.Result()
        result.result.status = ret
        self.get_logger().debug(f"Returning result {result}")

        self.get_logger().warn(
            f"Timestamp: {1000*(time.time() - start_time):.2f}ms at the end"
        )

        return result


def main(args=None):
    # rclpy.init(args=args)
    # r_arm_goto_action_server = GotoActionServer("r_arm")
    # rclpy.spin(r_arm_goto_action_server)
    # r_arm_goto_action_server.destroy_node()
    # rclpy.shutdown()

    rclpy.init(args=args)
    r_arm_goto_action_server = GotoActionServer("r_arm")

    mult_executor = MultiThreadedExecutor()

    mult_executor.add_node(r_arm_goto_action_server)
    executor_thread = threading.Thread(target=mult_executor.spin, daemon=True)
    executor_thread.start()
    rate = r_arm_goto_action_server.create_rate(2.0)

    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
    executor_thread.join()


if __name__ == "__main__":
    main()
