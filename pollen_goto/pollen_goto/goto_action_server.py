import time


from pollen_msgs.action import GotoTrajectory
from pollen_msgs.msg import TrajectoryFeedback, TrajectoryResult
from control_msgs.msg import DynamicJointState, InterfaceValue

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from .interpolation import InterpolationMode

from enum import Enum
from threading import Event
import threading
import numpy as np
from functools import partial
import collections


# TODO
class State(Enum):
    READY = 1
    RUNNING = 2


class GotoActionServer(Node):
    def __init__(self, name_prefix):
        super().__init__(f"{name_prefix}_goto_action_server")

        self._goal_queue = collections.deque()
        self._goal_queue_lock = threading.Lock()
        self._current_goal = None

        self._action_server = ActionServer(
            self,
            GotoTrajectory,
            f"{name_prefix}_goto",
            handle_accepted_callback=self.handle_accepted_callback,
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.joint_state = {}
        self.joint_state_ready = Event()
        self.joint_state_sub = self.create_subscription(
            msg_type=DynamicJointState,
            topic="/dynamic_joint_states",
            qos_profile=5,
            callback=self.on_dynamic_joint_states,
        )

        self.joint_commands_pub = self.create_publisher(
            msg_type=DynamicJointState,
            topic="/dynamic_joint_commands",
            qos_profile=5,
        )

        self.state = State.READY
        self.get_logger().info("Goto action server init.")

    def on_dynamic_joint_states(self, state: DynamicJointState):
        """Retreive the joint state from /dynamic_joint_states."""
        if not self.joint_state_ready.is_set():
            for uid, name in enumerate(state.joint_names):
                self.joint_state[name] = {}
                self.joint_state[name]["name"] = name
                self.joint_state[name]["uid"] = uid

        for uid, (name, kv) in enumerate(
            zip(state.joint_names, state.interface_values)
        ):
            for k, v in zip(kv.interface_names, kv.values):
                self.joint_state[name][k] = v

        if not self.joint_state_ready.is_set():
            for name, state in self.joint_state.items():
                if "position" in state:
                    state["target_position"] = state["position"]

            self.joint_state_ready.set()

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        # This server allows multiple goals in parallel
        self.get_logger().info(f"Received goal request: {goal_request}")
        # TODO CHECK
        traj = goal_request.trajectory
        if len(traj.joint_names) < 1 or (len(traj.joint_names) != len(traj.points)):
            self.get_logger().error(
                f"Invalid goal. Nb joint names={len(traj.joint_names)}, nb points={len(traj.points)}"
            )
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        """Start or defer execution of an already accepted goal."""

        with self._goal_queue_lock:
            if self._current_goal is not None:
                # Put incoming goal in the queue
                self._goal_queue.append(goal_handle)
                self.get_logger().info(f"New goal received and put in the queue")
            else:
                # Start goal execution right away
                self._current_goal = goal_handle
                self.get_logger().info(f"Empty queue, start executing")
                self._current_goal.execute()

    def check(self):
        pass

    def compute_traj(
        self,
        starting_pos,
        goal_pos,
        duration,
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

    def prepare_data(self, goal_msg):
        traj = goal_msg.trajectory
        goal_pos_dict = {}
        goal_vel_dict = {}
        goal_acc_dict = {}

        start_pos_dict = {}
        start_vel_dict = {}
        start_acc_dict = {}
        duration = None

        if self.joint_state_ready.is_set():
            for it, joint_name in enumerate(traj.joint_names):
                if duration is None:  # not set
                    duration = (
                        traj.points[it].time_from_start.sec
                        + traj.points[it].time_from_start.nanosec * 1e-9
                    )  # it would have cost only one line of code not to annoy hundreds of developpers but no https://github.com/ros2/rclpy/pull/1010

                goal_pos_dict[joint_name] = traj.points[it].positions[0]

                start_pos_dict[joint_name] = self.joint_state[joint_name]["position"]

                if len(traj.points[it].velocities) > 0:
                    goal_vel_dict[joint_name] = traj.points[it].velocities[0]
                    start_vel_dict[joint_name] = self.joint_state[joint_name][
                        "velocity"
                    ]

                else:
                    goal_vel_dict[joint_name] = 0.0
                    start_vel_dict[joint_name] = 0.0

                if len(traj.points[it].accelerations) > 0:
                    goal_acc_dict[joint_name] = 0.0  # traj.points[it].accelerations[0]
                    start_acc_dict[joint_name] = 0.0  # we don't have that...
                else:
                    goal_acc_dict[joint_name] = 0.0
                    start_acc_dict[joint_name] = 0.0

            return (
                start_pos_dict,
                goal_pos_dict,
                duration,
                start_vel_dict,
                start_acc_dict,
                goal_vel_dict,
                goal_acc_dict,
            )
        else:
            return None

    def cmd_pub(self, joints, points):
        cmd_msg = DynamicJointState()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        # cmd_msg.joint_names=joints

        for j, p in zip(joints, points):
            cmd_msg.joint_names.append(j)
            inter = InterfaceValue()
            inter.interface_names.append("position")
            inter.values.append(p)
            cmd_msg.interface_values.append(inter)

        self.joint_commands_pub.publish(cmd_msg)

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
        while True:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("Goal canceled")
                return "canceled"

            elapsed_time = self.get_clock().now() - t0
            elapsed_time = elapsed_time.nanoseconds * 1e-9

            if elapsed_time > duration:
                self.get_logger().info(f"goto finished")
                break

            point = traj_func(elapsed_time)

            feedback_msg = GotoTrajectory.Feedback()
            feedback_msg.feedback.status = "running"
            feedback_msg.feedback.time_to_completion = duration - elapsed_time

            self.cmd_pub(joints, point)
            goal_handle.publish_feedback(feedback_msg)
            rate.sleep()

        return "finished"

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info(f"Received cancel request")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Execute a goal."""

        try:
            self.get_logger().warn(f"Executing goal...")
            ret = ""
            if self.joint_state_ready.is_set():
                (
                    start_pos_dict,
                    goal_pos_dict,
                    duration,
                    start_vel_dict,
                    start_acc_dict,
                    goal_vel_dict,
                    goal_acc_dict,
                ) = self.prepare_data(goal_handle.request)
                self.get_logger().warn(
                    f"start_pos: {start_pos_dict} goal_pos: {goal_pos_dict} duration: {duration} start_vel: {start_vel_dict} start_acc: {start_acc_dict} goal_vel: {goal_vel_dict} goal_acc: {goal_acc_dict}"
                )
                traj_func = self.compute_traj(
                    np.array(list(start_pos_dict.values())),
                    np.array(list(goal_pos_dict.values())),
                    duration,
                    np.array(list(start_vel_dict.values())),
                    np.array(list(goal_vel_dict.values())),
                    np.array(list(start_acc_dict.values())),
                    np.array(list(goal_acc_dict.values())),
                )
                joints = start_pos_dict.keys()
                ret = self.goto(traj_func, joints, duration, goal_handle)

                goal_handle.succeed()

                # Populate result message
                result = GotoTrajectory.Result()
                result.result.status = ret
                self.get_logger().warn(f"Returning result {result}")

                return result

            else:
                self.get_logger().warn("Not ready...")
                # TODO
                goal_handle.abort()

                # Populate result message
                result = GotoTrajectory.Result()
                result.result.status = "failed"
                self.get_logger().warn(f"Returning failed result {result}")

                return result
        finally:
            with self._goal_queue_lock:
                try:
                    # Start execution of the next goal in the queue.
                    self._current_goal = self._goal_queue.popleft()
                    self.get_logger().warn(f"Next goal pulled from the queue")
                    self._current_goal.execute()
                except IndexError:
                    # No goal in the queue.
                    self._current_goal = None


def main(args=None):
    # Same as main but creating 2 nodes
    rclpy.init(args=args)
    r_arm_goto_action_server = GotoActionServer("r_arm")
    l_arm_goto_action_server = GotoActionServer("l_arm")
    neck_goto_action_server = GotoActionServer("neck")
    mult_executor = MultiThreadedExecutor()
    mult_executor.add_node(r_arm_goto_action_server)
    mult_executor.add_node(l_arm_goto_action_server)
    mult_executor.add_node(neck_goto_action_server)
    executor_thread = threading.Thread(target=mult_executor.spin, daemon=True)
    executor_thread.start()
    rate = r_arm_goto_action_server.create_rate(0.5)

    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
    executor_thread.join()


if __name__ == "__main__":
    main()
