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


class CentralJointStateHandler(Node):
    def __init__(self, shared_callback_group):
        super().__init__("central_joint_state_handler")
        self.joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.on_joint_state,
            5,
            callback_group=shared_callback_group,
        )
        self.dynamic_joint_commands_pub = self.create_publisher(
            msg_type=DynamicJointState,
            topic="/dynamic_joint_commands",
            qos_profile=5,
            callback_group=shared_callback_group,
        )
        self.joint_state = {}
        self.joint_state_ready = Event()

        # Not my proudest moment
        while not self.joint_state_ready.is_set():
            # spin once
            rclpy.spin_once(self, timeout_sec=0.1)
            self.get_logger().warn("Waiting for joint_state_ready to be set")
            time.sleep(0.1)

        self.dynamic_joint_commands = self.init_dynamic_joint_commands()
        self.timer = self.create_timer(1.0 / 150.0, self.publish_dynamic_joint_commands)

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


class GotoActionServer(Node):
    def __init__(self, name_prefix, joint_state_handler, shared_callback_group):
        super().__init__(f"{name_prefix}_goto_action_server")
        self.joint_state_handler = joint_state_handler
        self._goal_queue = collections.deque()
        self._goal_queue_lock = threading.Lock()
        self._current_goal = None

        self._action_server = ActionServer(
            self,
            Goto,
            f"{name_prefix}_goto",
            handle_accepted_callback=self.handle_accepted_callback,
            execute_callback=self.execute_callback,
            callback_group=shared_callback_group,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        # Not sending the feedback every tick
        self.nb_commands_per_feedback = 10
        self.get_logger().info("Goto action server init.")

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        # This server allows multiple goals in parallel
        self.get_logger().debug(f"Received goal request: {goal_request.request}")
        request = goal_request.request  # This is of type pollen_msgs/GotoRequest
        duration = request.duration
        if duration <= 0.001:
            self.get_logger().error(f"Invalid duration {duration}")
            return GoalResponse.REJECT

        elif len(request.goal_joints.name) < 1 or (
            len(request.goal_joints.name) != len(request.goal_joints.position)
        ):
            self.get_logger().error(
                f"Invalid goal. Nb joint names={len(request.goal_joints.name)}, nb positions={len(request.goal_joints.position)}"
            )
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        """Start or defer execution of an already accepted goal."""
        # print size of queue
        self.get_logger().info(f"Queue size: {len(self._goal_queue)}")
        # check time spent waiting here
        t = time.time()
        should_execute = False
        with self._goal_queue_lock:
            self.get_logger().info(f"Spent {1000*(time.time()-t)}ms waiting for lock")
            if self._current_goal is not None:
                # Put incoming goal in the queue
                self._goal_queue.append(goal_handle)
                self.get_logger().debug(f"New goal received and put in the queue")
            else:
                # Start goal execution right away
                self._current_goal = goal_handle
                self.get_logger().debug(f"Empty queue, start executing")
                should_execute = True
        if should_execute:
            self._current_goal.execute()

    def check(self):
        pass

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

            start_pos_dict[joint_name] = self.joint_state_handler.joint_state[
                joint_name
            ]["position"]

            if len(goto_request.goal_joints.velocity) > 0:
                goal_vel_dict[joint_name] = goto_request.goal_joints.velocity[it]
                start_vel_dict[joint_name] = self.joint_state_handler.joint_state[
                    joint_name
                ]["velocity"]

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
        # self.get_logger().error(
        #     f"{self.joint_state_handler.dynamic_joint_commands.interface_values[0].interface_names[0]}"
        # )
        # self.get_logger().error(
        #     f"{self.joint_state_handler.dynamic_joint_commands.interface_values[0].values[0]}"
        # )
        for name, p in zip(joints, points):
            # Find index of name in self.joint_state_handler.dynamic_joint_commands.joint_names
            idx = self.joint_state_handler.dynamic_joint_commands.joint_names.index(
                name
            )
            if idx == -1:
                self.get_logger().error(f"Joint {name} not found in joint_names")
                return
            # print self.joint_state_handler.dynamic_joint_commands
            self.joint_state_handler.dynamic_joint_commands.interface_values[
                idx
            ].values[0] = p

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

            self.cmd_pub(joints, point)
            commands_sent += 1

            if commands_sent % self.nb_commands_per_feedback == 0:
                feedback_msg = Goto.Feedback()
                feedback_msg.feedback.status = "running"
                feedback_msg.feedback.commands_sent = commands_sent
                feedback_msg.feedback.time_to_completion = duration - elapsed_time
                goal_handle.publish_feedback(feedback_msg)
            rate.sleep()

        return "finished"

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info(f"Received cancel request")

        # Check state and decide
        if goal_handle.is_active or goal_handle.is_executing:
            return CancelResponse.ACCEPT
        else:
            return CancelResponse.REJECT

    def execute_callback(self, goal_handle):
        """Execute a goal."""

        start_time = time.time()
        try:
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

            if mode == "linear":
                interpolation_mode = InterpolationMode.LINEAR
            elif mode == "minimum_jerk":
                interpolation_mode = InterpolationMode.MINIMUM_JERK
            else:
                self.get_logger().warn(
                    f"Unknown interpolation mode {mode} defaulting to minimum_jerk"
                )
                interpolation_mode = InterpolationMode.MINIMUM_JERK

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
                interpolation_mode=interpolation_mode,
            )
            self.get_logger().warn(
                f"Timestamp: {1000*(time.time() - start_time):.2f}ms after compute_traj"
            )
            joints = start_pos_dict.keys()
            ret = self.goto(
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

            # This is handled in the cancel_callback
            # elif ret == "canceled":
            #     goal_handle.canceled()

            # Populate result message
            result = Goto.Result()
            result.result.status = ret
            self.get_logger().debug(f"Returning result {result}")

            self.get_logger().warn(
                f"Timestamp: {1000*(time.time() - start_time):.2f}ms at the end"
            )

            return result

        finally:
            with self._goal_queue_lock:
                self.get_logger().warn(
                    f"Timestamp: {1000*(time.time() - start_time):.2f}ms after lock"
                )
                self._current_goal = None
                # check if _goal_queue has something in it
                if len(self._goal_queue) > 0:
                    self._current_goal = self._goal_queue.popleft()
                    self.get_logger().info(f"Next goal pulled from the queue")
            self.get_logger().warn(
                f"Timestamp: {1000*(time.time() - start_time):.2f}ms before next execute"
            )
            if self._current_goal is not None:
                self._current_goal.execute()


def main(args=None):
    # Same as main but creating 2 nodes
    rclpy.init(args=args)
    callback_group = ReentrantCallbackGroup()
    # callback_group1 = MutuallyExclusiveCallbackGroup()
    # callback_group2 = MutuallyExclusiveCallbackGroup()
    joint_state_handler = CentralJointStateHandler(callback_group)
    r_arm_goto_action_server = GotoActionServer(
        "r_arm", joint_state_handler, callback_group
    )
    l_arm_goto_action_server = GotoActionServer(
        "l_arm", joint_state_handler, callback_group
    )
    neck_goto_action_server = GotoActionServer(
        "neck", joint_state_handler, callback_group
    )
    mult_executor = MultiThreadedExecutor()
    mult_executor.add_node(joint_state_handler)
    mult_executor.add_node(r_arm_goto_action_server)
    mult_executor.add_node(l_arm_goto_action_server)
    mult_executor.add_node(neck_goto_action_server)
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
