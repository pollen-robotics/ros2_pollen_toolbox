import threading
import time
from queue import Queue
from threading import Event
from typing import Optional

import numpy as np
import rclpy
from control_msgs.msg import DynamicJointState, InterfaceValue
from geometry_msgs.msg import PoseStamped
from pollen_msgs.action import Goto
from pollen_msgs.msg import IKRequest
from pollen_msgs.srv import GetForwardKinematics, GetInverseKinematics
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup  # ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from .interpolation import CartesianSpaceInterpolationMode, JointSpaceInterpolationMode


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
        self.active_goals_lock = threading.Lock()
        self.active_goals = 0

        self.joint_state = {}
        self.joint_state_ready = Event()

        # I'd rather have this than the locks for something that happens only once at startup
        while not self.joint_state_ready.is_set():
            rclpy.spin_once(self, timeout_sec=0.1)
            self.get_logger().warn("Waiting for joint_state_ready to be set")
            time.sleep(0.1)

        self.dynamic_joint_commands = self.init_dynamic_joint_commands()

    def increment_active_goals(self):
        with self.active_goals_lock:
            self.active_goals += 1

    def decrement_active_goals(self):
        with self.active_goals_lock:
            self.active_goals -= 1

    def has_active_goals(self):
        with self.active_goals_lock:
            return self.active_goals > 0

    def on_joint_state(self, state: JointState):
        """Retreive the joint state from /joint_states."""
        if not self.joint_state_ready.is_set():
            for uid, name in enumerate(state.name):
                self.joint_state[name] = {}
            self.joint_state_ready.set()

        for name, pos, vel, effort in zip(state.name, state.position, state.velocity, state.effort):
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


class GotoActionServer(Node):
    def __init__(self, name_prefix, joint_state_handler, shared_callback_group, init_kinematics=False):
        super().__init__(f"{name_prefix}_goto_action_server")
        self.joint_state_handler = joint_state_handler
        self._goal_queue = Queue()
        self.execution_ongoing = Event()

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

        self.dynamic_joint_commands_pub = self.create_publisher(
            msg_type=DynamicJointState,
            topic="/dynamic_joint_commands",
            qos_profile=5,
            callback_group=shared_callback_group,
        )

        if init_kinematics:
            high_freq_qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,  # Prioritizes speed over guaranteed delivery
                history=HistoryPolicy.KEEP_LAST,  # Keeps only a fixed number of messages
                depth=1,  # Minimal depth, for the latest message
                # Other QoS settings can be adjusted as needed
            )

            self.arm_target_pose_pub = self.create_publisher(
                msg_type=IKRequest,
                topic=f"/{name_prefix}/ik_target_pose",
                qos_profile=high_freq_qos_profile,
            )

            if name_prefix == "neck":
                name_prefix = "head"

            self.forward_sub = self.create_client(
                srv_type=GetForwardKinematics,
                srv_name=f"/{name_prefix}/forward_kinematics",
            )
            self.forward_sub.wait_for_service()

        # Not sending the feedback every tick
        self.nb_commands_per_feedback = 10
        self.get_logger().info(f"Goto action server {name_prefix} init.")
        # create thread for check_queue_and_execute
        self.check_queue_and_execute_thread = threading.Thread(target=self.check_queue_and_execute, daemon=True)
        self.check_queue_and_execute_thread.start()

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
            self.get_logger().debug(f"Invalid duration {duration}")
            return GoalResponse.REJECT

        elif request.interpolation_space == "joints" and (
            len(request.goal_joints.name) < 1 or (len(request.goal_joints.name) != len(request.goal_joints.position))
        ):
            self.get_logger().debug(
                f"Invalid goal. Nb joint names={len(request.goal_joints.name)}, nb positions={len(request.goal_joints.position)}"
            )
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        self._goal_queue.put(goal_handle)

    def check_queue_and_execute(self):
        while True:
            goal_handle = self._goal_queue.get()
            self.execution_ongoing.clear()
            self.joint_state_handler.increment_active_goals()
            goal_handle.execute()
            self.execution_ongoing.wait()
            self.joint_state_handler.decrement_active_goals()

    def check(self):
        pass

    def compute_traj_joint_space(
        self,
        duration,
        starting_pos,
        goal_pos,
        starting_vel=None,
        goal_vel=None,
        starting_acc=None,
        goal_acc=None,
        interpolation_mode: JointSpaceInterpolationMode = JointSpaceInterpolationMode.MINIMUM_JERK,
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

    def compute_traj_cartesian_space(
        self,
        duration,
        starting_pose,
        goal_pose,
        interpolation_mode: CartesianSpaceInterpolationMode = CartesianSpaceInterpolationMode.LINEAR,
        arc_direction: Optional[str] = "above",
        secondary_radius: Optional[float] = None,
    ):
        return interpolation_mode(
            starting_pose,
            goal_pose,
            duration,
            arc_direction,
            secondary_radius,
        )

    def prepare_data_joint_space(self, goto_request):
        # goto_request is of type pollen_msgs/GotoRequest
        goal_pos_dict = {}
        goal_vel_dict = {}
        goal_acc_dict = {}

        start_pos_dict = {}
        start_vel_dict = {}
        start_acc_dict = {}

        for it, joint_name in enumerate(goto_request.goal_joints.name):
            goal_pos_dict[joint_name] = goto_request.goal_joints.position[it]

            start_pos_dict[joint_name] = self.joint_state_handler.joint_state[joint_name]["position"]

            if len(goto_request.goal_joints.velocity) > 0:
                goal_vel_dict[joint_name] = goto_request.goal_joints.velocity[it]
                start_vel_dict[joint_name] = self.joint_state_handler.joint_state[joint_name]["velocity"]

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

    def prepare_data_cartesian_space(self, goto_request):
        js = JointState()
        for joint_name in goto_request.goal_joints.name:
            js.name.append(joint_name)
            js.position.append(self.joint_state_handler.joint_state[joint_name]["position"])

        req = GetForwardKinematics.Request()
        req.joint_position = js

        resp = self.forward_sub.call(req)

        start_pose = PoseStamped()
        start_pose.pose = resp.pose

        if resp.success:
            return (
                start_pose,
                goto_request.goal_pose,
            )

    def cmd_joints_pub(self, joints, points):
        cmd_msg = DynamicJointState()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()

        for j, p in zip(joints, points):
            cmd_msg.joint_names.append(j)
            inter = InterfaceValue()
            inter.interface_names.append("position")
            inter.values.append(p)
            cmd_msg.interface_values.append(inter)

        self.dynamic_joint_commands_pub.publish(cmd_msg)

    def cmd_pose_pub(self, pose):
        req = IKRequest()
        req.pose = pose
        req.pose.header.stamp = self.get_clock().now().to_msg()

        req.constrained_mode = "unconstrained"
        req.continuous_mode = "continuous"
        req.preferred_theta = -4 * np.pi / 6
        req.d_theta_max = 0.01
        req.order_id = 0

        self.arm_target_pose_pub.publish(req)

    def get_joint_indices(self, joints):
        indices = []
        for name in joints:
            try:
                idx = self.joint_state_handler.dynamic_joint_commands.joint_names.index(name)
                indices.append(idx)
            except ValueError:
                self.get_logger().error(f"Joint {name} not found in joint_names")
                raise
        return indices

    def goto_time(self, traj_func, joints, duration, goal_handle, interpolation_space: str, sampling_freq=100):
        length = round(duration * sampling_freq)
        if length < 1:
            raise ValueError(f"Goto length too short! (incoherent duration {duration} or sampling_freq {sampling_freq})!")

        t0 = time.time()
        dt = 1 / sampling_freq

        commands_sent = 0
        while True:
            # t0_loop = self.get_clock().now()
            t0_loop = time.time()
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("Goal canceled")
                return "canceled"

            # elapsed_time = t0_loop - t0
            # elapsed_time = elapsed_time.nanoseconds * 1e-9
            elapsed_time = time.time() - t0

            if elapsed_time > duration:
                # Send the last point to increase precision and break
                # We're calling the traj_function on a time > duration on purpose,
                # as it's coded to return the goal position when t > duration
                point = traj_func(elapsed_time)

                if interpolation_space == "joints":
                    self.cmd_joints_pub(joints, point)
                elif interpolation_space == "cartesian":
                    self.cmd_pose_pub(point)
                self.get_logger().info(f"goto finished")
                break

            point = traj_func(elapsed_time)

            if interpolation_space == "joints":
                self.cmd_joints_pub(joints, point)
                commands_sent += 1
            elif interpolation_space == "cartesian":
                self.cmd_pose_pub(point)
                commands_sent += 1

            if commands_sent % self.nb_commands_per_feedback == 0:
                feedback_msg = Goto.Feedback()
                feedback_msg.feedback.status = "running"
                feedback_msg.feedback.commands_sent = commands_sent
                feedback_msg.feedback.time_to_completion = duration - elapsed_time
                goal_handle.publish_feedback(feedback_msg)

            # self.rate.sleep()  # Slowly the output freq drops with this...

            # Calculate the time to sleep to achieve the desired frequency
            # elapsed_loop = self.get_clock().now() - t0
            # elapsed_loop = elapsed_loop.nanoseconds * 1e-9
            # sleep_duration = max(0, dt - elapsed_loop)
            # # TODO do better. But Rate() won't work well in this context.
            # time.sleep(sleep_duration)
            time.sleep(max(0, dt - (time.time() - t0_loop)))

        return "finished"

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info(f"Received cancel request")

        # Check state and decide
        if goal_handle.is_active or goal_handle.is_executing:
            return CancelResponse.ACCEPT
        else:
            return CancelResponse.REJECT

    def callback_for_joint_space(self, goal_handle, interpolation_mode):
        start_time = time.time()

        goto_request = goal_handle.request.request  # pollen_msgs/GotoRequest
        duration = goto_request.duration
        sampling_freq = goto_request.sampling_freq
        safety_on = goto_request.safety_on

        # try:
        self.get_logger().info(f"Executing goal...")
        ret = ""
        self.get_logger().debug(f"Timestamp: {1000*(time.time() - start_time):.2f}ms after joint_state_ready.is_set()")

        (
            start_pos_dict,
            goal_pos_dict,
            start_vel_dict,
            goal_vel_dict,
            start_acc_dict,
            goal_acc_dict,
        ) = self.prepare_data_joint_space(goto_request)

        self.get_logger().debug(f"Timestamp: {1000*(time.time() - start_time):.2f}ms after prepare data")
        self.get_logger().debug(
            f"start_pos: {start_pos_dict} goal_pos: {goal_pos_dict} duration: {duration} start_vel: {start_vel_dict} start_acc: {start_acc_dict} goal_vel: {goal_vel_dict} goal_acc: {goal_acc_dict}"
        )

        traj_func = self.compute_traj_joint_space(
            goto_request.duration,
            np.array(list(start_pos_dict.values())),
            np.array(list(goal_pos_dict.values())),
            np.array(list(start_vel_dict.values())),
            np.array(list(goal_vel_dict.values())),
            np.array(list(start_acc_dict.values())),
            np.array(list(goal_acc_dict.values())),
            interpolation_mode=interpolation_mode,
        )

        self.get_logger().debug(f"Timestamp: {1000*(time.time() - start_time):.2f}ms after compute_traj")

        joints = start_pos_dict.keys()

        ret = self.goto_time(
            traj_func,
            joints,
            duration,
            goal_handle,
            interpolation_space="joints",
            sampling_freq=sampling_freq,
        )
        self.get_logger().debug(f"Timestamp: {1000*(time.time() - start_time):.2f}ms after goto")

        return ret

    def callback_for_cartesian_space(self, goal_handle, interpolation_mode):
        start_time = time.time()

        goto_request = goal_handle.request.request  # pollen_msgs/GotoRequest
        duration = goto_request.duration
        sampling_freq = goto_request.sampling_freq
        safety_on = goto_request.safety_on

        # try:
        self.get_logger().info(f"Executing goal...")
        ret = ""
        self.get_logger().debug(f"Timestamp: {1000*(time.time() - start_time):.2f}ms after joint_state_ready.is_set()")

        (
            start_pose,
            goal_pose,
        ) = self.prepare_data_cartesian_space(goto_request)

        traj_func = self.compute_traj_cartesian_space(
            goto_request.duration,
            start_pose,
            goal_pose,
            interpolation_mode=interpolation_mode,
            arc_direction=goto_request.arc_direction,
            secondary_radius=goto_request.secondary_radius,
        )

        ret = self.goto_time(
            traj_func,
            None,
            duration,
            goal_handle,
            interpolation_space="cartesian",
            sampling_freq=sampling_freq,
        )
        self.get_logger().debug(f"Timestamp: {1000*(time.time() - start_time):.2f}ms after goto")

        return ret

    def execute_callback(self, goal_handle):
        """Execute a goal."""
        goto_request = goal_handle.request.request  # pollen_msgs/GotoRequest
        interpolation_space = goto_request.interpolation_space
        mode = goto_request.mode

        if interpolation_space == "joints":
            if mode == "linear":
                interpolation_mode = JointSpaceInterpolationMode.LINEAR
            elif mode == "minimum_jerk":
                interpolation_mode = JointSpaceInterpolationMode.MINIMUM_JERK
            else:
                self.get_logger().warn(f"Unknown interpolation mode {mode} defaulting to minimum_jerk")
                interpolation_mode = JointSpaceInterpolationMode.MINIMUM_JERK
            callback = self.callback_for_joint_space

        elif interpolation_space == "cartesian":
            if mode == "linear":
                interpolation_mode = CartesianSpaceInterpolationMode.LINEAR
            elif mode == "minimum_jerk":
                interpolation_mode = CartesianSpaceInterpolationMode.MINIMUM_JERK
            elif mode == "elliptical":
                interpolation_mode = CartesianSpaceInterpolationMode.ELLIPTICAL
            else:
                self.get_logger().warn(f"Unknown interpolation mode {mode} defaulting to linear")
                interpolation_mode = CartesianSpaceInterpolationMode.LINEAR
            callback = self.callback_for_cartesian_space

        else:
            self.get_logger().warn(f"Unknown interpolation space {interpolation_space} defaulting to joints")
            interpolation_mode = JointSpaceInterpolationMode.MINIMUM_JERK
            callback = self.callback_for_joint_space

        ret = callback(goal_handle, interpolation_mode)

        if ret == "finished":
            goal_handle.succeed()

        # Populate result message
        result = Goto.Result()
        result.result.status = ret
        self.get_logger().debug(f"Returning result {result}")

        self.execution_ongoing.set()
        return result


def main(args=None):
    rclpy.init(args=args)
    # callback_group = ReentrantCallbackGroup()
    callback_group = MutuallyExclusiveCallbackGroup()

    joint_state_handler = CentralJointStateHandler(callback_group)
    r_arm_goto_action_server = GotoActionServer("r_arm", joint_state_handler, callback_group, init_kinematics=True)
    l_arm_goto_action_server = GotoActionServer("l_arm", joint_state_handler, callback_group, init_kinematics=True)
    neck_goto_action_server = GotoActionServer("neck", joint_state_handler, callback_group, init_kinematics=True)
    antenna_right_goto_action_server = GotoActionServer("antenna_right", joint_state_handler, callback_group)
    antenna_left_goto_action_server = GotoActionServer("antenna_left", joint_state_handler, callback_group)
    mult_executor = MultiThreadedExecutor()
    mult_executor.add_node(joint_state_handler)
    mult_executor.add_node(r_arm_goto_action_server)
    mult_executor.add_node(l_arm_goto_action_server)
    mult_executor.add_node(neck_goto_action_server)
    mult_executor.add_node(antenna_right_goto_action_server)
    mult_executor.add_node(antenna_left_goto_action_server)
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
