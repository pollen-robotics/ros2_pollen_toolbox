import copy
import time
from functools import partial
from threading import Event
from typing import List

import numpy as np
import prometheus_client as prc
import rclpy
import reachy2_monitoring as rm
from geometry_msgs.msg import Pose, PoseStamped
from pollen_msgs.msg import CartTarget, IKRequest, ReachabilityState
from pollen_msgs.srv import GetForwardKinematics, GetInverseKinematics
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.qos import HistoryPolicy, QoSDurabilityPolicy, QoSProfile, ReliabilityPolicy
from reachy2_symbolic_ik.control_ik import ControlIK
from reachy2_symbolic_ik.symbolic_ik import SymbolicIK
from reachy2_symbolic_ik.utils import (
    allow_multiturn,
    get_best_continuous_theta,
    get_best_continuous_theta2,
    get_best_discrete_theta,
    get_best_theta_to_current_joints,
    get_euler_from_homogeneous_matrix,
    limit_orbita3d_joints,
    limit_orbita3d_joints_wrist,
    limit_theta_to_interval,
)
from reachy_config import ReachyConfig
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Header, String
from visualization_msgs.msg import MarkerArray

from pollen_grasping_utils.utils import get_grasp_marker

from .kdl_kinematics import (
    forward_kinematics,
    generate_solver,
    inverse_kinematics,
    ros_pose_to_matrix,
)
from .pose_averager import PoseAverager

SHOW_RVIZ_MARKERS = False

NODE_NAME = "pollen_kdl_kinematics_node"
rm.configure_pyroscope(
    NODE_NAME,
    tags={
        "server": "true",
        "client": "true",
    },
)


class PollenKdlKinematics(LifecycleNode):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.logger = self.get_logger()
        prc.start_http_server(10003)

        self.urdf = self.retrieve_urdf()

        self.tracer = rm.tracer(NODE_NAME)

        # Listen to /joint_state to get current position
        # used by averaged_target_pose
        self._current_pos = {}
        self.joint_state_sub = self.create_subscription(
            msg_type=JointState,
            topic="/joint_states",
            qos_profile=5,
            callback=self.on_joint_state,
        )
        self.joint_state_ready = Event()
        self.wait_for_joint_state()

        self.chain, self.fk_solver, self.ik_solver = {}, {}, {}
        self.fk_srv, self.ik_srv = {}, {}
        self.reachability_pub = {}
        self.target_sub, self.averaged_target_sub = {}, {}
        self.ik_target_sub = {}
        self.averaged_pose = {}
        self.max_joint_vel = {}
        self.prc_summaries = {}

        # High frequency QoS profile
        high_freq_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Prioritizes speed over guaranteed delivery
            history=HistoryPolicy.KEEP_LAST,  # Keeps only a fixed number of messages
            depth=1,  # Minimal depth, for the latest message
            # Other QoS settings can be adjusted as needed
        )

        self.orbita3D_max_angle = np.deg2rad(42.5)  # 43.5 is too much

        for prefix in ("l", "r"):
            arm = f"{prefix}_arm"
            part_name = arm
            self.prc_summaries[part_name] = prc.Summary(
                f"pollen_kdl_kinematics__on_ik_target_pose_{part_name}_time",
                f"Time spent during 'on_ik_target_pose' {part_name} callback",
            )

            chain, fk_solver, ik_solver = generate_solver(
                self.urdf, "torso", f"{prefix}_arm_tip"
            )

            # We automatically loads the kinematics corresponding to the config
            if chain.getNrOfJoints():
                self.logger.info(f'Found kinematics chain for "{arm}"!')

                for j in self.get_chain_joints_name(chain):
                    self.logger.info(f"\t{j}")

                # Create forward kinematics service
                self.fk_srv[arm] = self.create_service(
                    srv_type=GetForwardKinematics,
                    srv_name=f"/{arm}/forward_kinematics",
                    callback=partial(self.forward_kinematics_srv, name=arm),
                )
                self.logger.info(f'Adding service "{self.fk_srv[arm].srv_name}"...')

                # Create inverse kinematics service
                self.ik_srv[arm] = self.create_service(
                    srv_type=GetInverseKinematics,
                    srv_name=f"/{arm}/inverse_kinematics",
                    callback=partial(self.inverse_kinematics_srv, name=arm),
                )
                self.logger.info(f'Adding service "{self.ik_srv[arm].srv_name}"...')

                # Create cartesian control pub/subscription
                forward_position_pub = self.create_publisher(
                    msg_type=Float64MultiArray,
                    topic=f"/{arm}_forward_position_controller/commands",
                    qos_profile=5,
                )

                self.reachability_pub[arm] = self.create_publisher(
                    msg_type=ReachabilityState,
                    topic=f"/{arm}_reachability_states",
                    qos_profile=10,
                )

                if arm.startswith("l"):
                    q0 = [0.0, np.pi / 2, 0.0, -np.pi / 2, 0.0, 0.0, 0.0]
                else:
                    q0 = [0.0, -np.pi / 2, 0.0, -np.pi / 2, 0.0, 0.0, 0.0]

                # Keeping both on _target_pose mechanisms for compatibility reasons
                self.target_sub[arm] = self.create_subscription(
                    msg_type=PoseStamped,
                    topic=f"/{arm}/target_pose",
                    qos_profile=high_freq_qos_profile,
                    callback=partial(
                        self.on_target_pose,
                        name=arm,
                        # arm straight, with elbow at -90 (facing forward)
                        # q0=[0, 0, 0, -np.pi / 2, 0, 0, 0],
                        # The Coralie pose
                        q0=q0,
                        forward_publisher=forward_position_pub,
                    ),
                )
                self.logger.info(
                    f'Adding subscription on "{self.target_sub[arm].topic}"...'
                )

                self.ik_target_sub[arm] = self.create_subscription(
                    msg_type=IKRequest,
                    topic=f"/{arm}/ik_target_pose",
                    qos_profile=high_freq_qos_profile,
                    callback=partial(
                        self.on_ik_target_pose,
                        name=arm,
                        forward_publisher=forward_position_pub,
                    ),
                )
                self.logger.info(
                    f'Adding subscription on "{self.ik_target_sub[arm].topic}"...'
                )

                self.averaged_target_sub[arm] = self.create_subscription(
                    msg_type=PoseStamped,
                    topic=f"/{arm}/averaged_target_pose",
                    qos_profile=high_freq_qos_profile,
                    callback=partial(
                        self.on_averaged_target_pose,
                        name=arm,
                        # arm straight, with elbow at -90 (facing forward)
                        q0=[0, 0, 0, -np.pi / 2, 0, 0, 0],
                        forward_publisher=forward_position_pub,
                    ),
                )
                self.averaged_pose[arm] = PoseAverager(window_length=1)
                self.default_max_joint_vel = (
                    0.6  # Angular difference between current joint and requested joint.
                )
                self.max_joint_vel[arm] = np.array(
                    [
                        self.default_max_joint_vel,
                        self.default_max_joint_vel,
                        self.default_max_joint_vel,
                        self.default_max_joint_vel,
                        self.default_max_joint_vel,
                        self.default_max_joint_vel,
                        self.default_max_joint_vel,
                    ]
                )
                self.logger.info(
                    f'Adding subscription on "{self.target_sub[arm].topic}"...'
                )

                self.chain[arm] = chain
                self.fk_solver[arm] = fk_solver
                self.ik_solver[arm] = ik_solver

        # Kinematics for the head
        chain, fk_solver, ik_solver = generate_solver(
            self.urdf,
            "torso",
            "head_tip",
            L=np.array([1e-6, 1e-6, 1e-6, 1.0, 1.0, 1.0]),
        )  # L weight matrix to considere only the orientation

        # We automatically loads the kinematics corresponding to the config
        if chain.getNrOfJoints():
            self.logger.info(f"Found kinematics chain for head!")

            for j in self.get_chain_joints_name(chain):
                self.logger.info(f"\t{j}")

            # Create forward kinematics service
            srv = self.create_service(
                srv_type=GetForwardKinematics,
                srv_name="/head/forward_kinematics",
                callback=partial(self.forward_kinematics_srv, name="head"),
            )
            self.fk_srv["head"] = srv
            self.logger.info(f'Adding service "{srv.srv_name}"...')

            # Create inverse kinematics service
            srv = self.create_service(
                srv_type=GetInverseKinematics,
                srv_name="/head/inverse_kinematics",
                callback=partial(self.inverse_kinematics_srv, name="head"),
            )
            self.ik_srv["head"] = srv
            self.logger.info(f'Adding service "{srv.srv_name}"...')

            # Create cartesian control subscription
            head_forward_position_pub = self.create_publisher(
                msg_type=Float64MultiArray,
                topic="/neck_forward_position_controller/commands",  # need
                qos_profile=5,
            )

            sub = self.create_subscription(
                msg_type=PoseStamped,
                topic="/head/target_pose",
                qos_profile=high_freq_qos_profile,
                callback=partial(
                    self.on_target_pose,
                    name="head",
                    # head straight
                    q0=[0, 0, 0],
                    forward_publisher=head_forward_position_pub,
                ),
            )
            sub = self.create_subscription(
                msg_type=CartTarget,
                topic="/head/cart_target_pose",
                qos_profile=high_freq_qos_profile,
                callback=partial(
                    self.on_ik_target_pose_neck,
                    name="head",
                    # head straight
                    q0=[0, 0, 0],
                    forward_publisher=head_forward_position_pub,
                ),
            )
            self.target_sub["head"] = sub
            self.logger.info(f'Adding subscription on "{sub.topic}"...')

            sub = self.create_subscription(
                msg_type=PoseStamped,
                topic="/head/averaged_target_pose",
                qos_profile=5,
                callback=partial(
                    self.on_averaged_target_pose,
                    name="head",
                    # head straight
                    q0=[0, 0, 0],
                    forward_publisher=head_forward_position_pub,
                ),
            )
            self.averaged_target_sub["head"] = sub
            self.averaged_pose["head"] = PoseAverager(window_length=1)

            self.max_joint_vel["head"] = np.array(
                [
                    self.default_max_joint_vel,
                    self.default_max_joint_vel,
                    self.default_max_joint_vel,
                ]
            )
            self.logger.info(f'Adding subscription on "{sub.topic}"...')

            part_name = "head"
            self.prc_summaries[part_name] = prc.Summary(
                f"pollen_kdl_kinematics__on_ik_target_pose_{part_name}_time",
                f"Time spent during 'on_ik_target_pose' {part_name} callback",
            )
            self.chain["head"] = chain
            self.fk_solver["head"] = fk_solver
            self.ik_solver["head"] = ik_solver

        current_joints_r = self.get_current_position(self.chain["r_arm"])
        current_joints_l = self.get_current_position(self.chain["l_arm"])
        current_joints = [current_joints_r, current_joints_l]
        error, current_pose_r = forward_kinematics(
            self.fk_solver["r_arm"],
            current_joints_r,
            self.chain["r_arm"].getNrOfJoints(),
        )
        error, current_pose_l = forward_kinematics(
            self.fk_solver["l_arm"],
            current_joints_l,
            self.chain["l_arm"].getNrOfJoints(),
        )
        current_pose = [current_pose_r, current_pose_l]

        reachy_config = ReachyConfig(no_print=True)

        r_orbita3D_max_angle = reachy_config.config["right_wrist_poulpe3d"]['config']['orientation_limits']['orbita3D_max_angle']
        l_orbita3D_max_angle = reachy_config.config["left_wrist_poulpe3d"]['config']['orientation_limits']['orbita3D_max_angle']
        self.control_ik = ControlIK(
            logger=self.logger,
            current_joints=current_joints,
            current_pose=current_pose,
            urdf=self.urdf,
            orbita3D_max_angle=[r_orbita3D_max_angle, l_orbita3D_max_angle],
            reachy_model=reachy_config.model,
            is_dvt=reachy_config.dvt or reachy_config.pvt,
        )

        self.logger.info(f"Kinematics node ready!")
        self.trigger_configure()

        self._marker_pub = self.create_publisher(
            MarkerArray, "markers_grasp_triplet", 10
        )
        self.marker_array = MarkerArray()

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        # Dummy state to minimize impact on current behavior
        self.logger.info(
            "Configuring state has been called, going into inactive to release event trigger"
        )
        return TransitionCallbackReturn.SUCCESS

    def forward_kinematics_srv(
        self,
        request: GetForwardKinematics.Request,
        response: GetForwardKinematics.Response,
        name,
    ) -> GetForwardKinematics.Response:
        try:
            joint_position = self.check_position(
                request.joint_position, self.chain[name]
            )
        except KeyError:
            response.success = False
            return response

        error, sol = forward_kinematics(
            self.fk_solver[name],
            joint_position,
            self.chain[name].getNrOfJoints(),
        )

        x, y, z = sol[:3, 3]
        q = Rotation.from_matrix(sol[:3, :3]).as_quat()

        # TODO: use error?
        response.success = True

        response.pose.position.x = x
        response.pose.position.y = y
        response.pose.position.z = z

        response.pose.orientation.x = q[0]
        response.pose.orientation.y = q[1]
        response.pose.orientation.z = q[2]
        response.pose.orientation.w = q[3]

        return response

    def inverse_kinematics_srv(
        self,
        request: GetInverseKinematics.Request,
        response: GetInverseKinematics.Response,
        name,
    ) -> GetInverseKinematics.Response:
        # Publish goal pose marker to visualize in RViz
        marker_array = MarkerArray()

        grasp_markers = get_grasp_marker(
            header=Header(
                stamp=self.get_clock().now().to_msg(),
                frame_id="torso",
            ),
            grasp_pose=request.pose,
            marker_id=1,
            tip_length=0.1,  # GRASP_MARKER_TIP_LEN, taken from simple_grasp_pose.py
            width=40,  # GRASP_MARKER_WIDTH, taken from simple_grasp_pose.py
            score=1.0,
            color=[1.0, 0.0, 0.0, 1.0],
            lifetime=15.0,
        )
        marker_array.markers.extend(grasp_markers.markers)
        self._marker_pub.publish(marker_array)

        M = ros_pose_to_matrix(request.pose)
        q0 = request.q0.position

        current_joints = self.get_current_position(self.chain[name])
        error, current_pose = forward_kinematics(
            self.fk_solver[name],
            current_joints,
            self.chain[name].getNrOfJoints(),
        )
        current_pose = np.array(current_pose)
        # self.logger.info(f"Current pose: {current_pose}")

        if "arm" in name:
            sol, is_reachable, state = self.control_ik.symbolic_inverse_kinematics(
                name,
                M,
                "discrete",
                current_joints=current_joints,
                constrained_mode="unconstrained",
                current_pose=current_pose,
            )

            # # self.logger.info(M)
            # self.logger.info(f"solution {sol} {is_reachable}")
        else:
            error, sol = inverse_kinematics(
                self.ik_solver[name],
                q0=q0,
                target_pose=M,
                nb_joints=self.chain[name].getNrOfJoints(),
            )
            sol = limit_orbita3d_joints(sol, self.orbita3D_max_angle)
            is_reachable = True

        response.success = is_reachable
        response.joint_position.name = self.get_chain_joints_name(self.chain[name])
        response.joint_position.position = sol

        # self.logger.info(f"{sol} (KDL solution)")

        return response

    def on_ik_target_pose_neck(self, msg: CartTarget, name, q0, forward_publisher):
        if "arm" in name:
            self.logger.error("IK target pose neck should be only for the neck")
            raise ValueError("IK target pose neck should be only for the neck")

        M = ros_pose_to_matrix(msg.pose.pose)
        ctx = rm.ctx_from_traceparent(msg.traceparent)

        trace_name = f"{name}::on_ik_target_pose_neck"
        rm.travel_span(
            f"{trace_name}_msg_travel",
            start_time=rclpy.time.Time.from_msg(msg.pose.header.stamp).nanoseconds,
            tracer=self.tracer,
            context=ctx,
        )

        with rm.PollenSpan(
            tracer=self.tracer,
            trace_name=trace_name,
            with_pyroscope=True,
            pyroscope_tags={"trace_name": trace_name},
            kind=rm.trace.SpanKind.SERVER,
            context=ctx,
        ) as stack, self.prc_summaries[name].time():
            stack.span.set_attributes(
                {
                    "rpc.system": "ros_topic",
                    "server.address": "localhost",
                    "q0": q0,
                    "M": M.astype(float).flatten().tolist(),
                }
            )

            error, sol = inverse_kinematics(
                self.ik_solver[name],
                q0=q0,
                target_pose=M,
                nb_joints=self.chain[name].getNrOfJoints(),
            )
            sol = limit_orbita3d_joints(sol, self.orbita3D_max_angle)
            stack.span.set_attributes({"sol": sol, "error": error})

            msg = Float64MultiArray()
            msg.data = sol
            forward_publisher.publish(msg)

    def on_ik_target_pose(self, msg: IKRequest, name, forward_publisher):

        ctx = rm.ctx_from_traceparent(msg.traceparent)

        trace_name = f"{name}::on_ik_target_pose"
        rm.travel_span(
            f"{trace_name}_msg_travel",
            start_time=rclpy.time.Time.from_msg(msg.pose.header.stamp).nanoseconds,
            tracer=self.tracer,
            context=ctx,
        )

        with rm.PollenSpan(
            tracer=self.tracer,
            trace_name=trace_name,
            with_pyroscope=True,
            pyroscope_tags={"trace_name": trace_name},
            kind=rm.trace.SpanKind.SERVER,
            context=ctx,
        ) as stack, self.prc_summaries[name].time():
            M = ros_pose_to_matrix(msg.pose.pose)

            stack.span.set_attributes(
                {
                    "rpc.system": "ros_topic",
                    "server.address": "localhost",
                    "M": M.astype(float).flatten().tolist(),
                    "control_ik.last_call_t1[name]": str(
                        self.control_ik.last_call_t[name]
                    ),
                }
            )

            constrained_mode = msg.constrained_mode
            continuous_mode = msg.continuous_mode
            preferred_theta = msg.preferred_theta
            d_theta_max = msg.d_theta_max
            order_id = msg.order_id

            if continuous_mode == "undefined":
                continuous_mode = "continuous"

            if constrained_mode == "undefined":
                constrained_mode = "unconstrained"

            # if constrained_mode == "low_elbow":
            #     interval_limit = [-4 * np.pi / 5, 0]
            # self.logger.info(f"Preferred theta: {preferred_theta}")
            # self.logger.info(f"Continuous mode: {continuous_mode}")
            # self.logger.info(f"Constrained mode: {constrained_mode}")
            # self.logger.info(f"Order ID: {order_id}")
            # self.logger.info(f"Max d_theta: {d_theta_max}")
            if name == "r_arm":
                marker_id = 10
            else:
                marker_id = 30
            if SHOW_RVIZ_MARKERS:
                if name == "r_arm":
                    self.marker_array = MarkerArray()
                grasp_markers = get_grasp_marker(
                    header=Header(
                        stamp=self.get_clock().now().to_msg(),
                        frame_id="torso",
                    ),
                    grasp_pose=msg.pose.pose,
                    marker_id=marker_id,
                    tip_length=0.1,  # GRASP_MARKER_TIP_LEN, taken from simple_grasp_pose.py
                    width=40,  # GRASP_MARKER_WIDTH, taken from simple_grasp_pose.py
                    score=1.0,
                    color=[1.0, 0.0, 0.0, 1.0],
                    lifetime=15.0,
                )
                self.marker_array.markers.extend(grasp_markers.markers)

            if "arm" in name:
                current_joints = self.get_current_position(self.chain[name])
                error, current_pose = forward_kinematics(
                    self.fk_solver[name],
                    current_joints,
                    self.chain[name].getNrOfJoints(),
                )

                sol, is_reachable, state = self.control_ik.symbolic_inverse_kinematics(
                    name,
                    M,
                    continuous_mode,
                    current_joints=current_joints,
                    constrained_mode="unconstrained",
                    current_pose=current_pose,
                    d_theta_max=0.02,
                    preferred_theta=preferred_theta,
                )

                # self.logger.info(f" solution {sol} {is_reachable} name {name}")
                goal_pose = self.control_ik.symbolic_ik_solver[name].goal_pose
                # self.logger.info(f"wrist pose: {self.control_ik.symbolic_ik_solver['r_arm'].wrist_position}")
                # self.logger.info(f"goal pose: {goal_pose}")
                pose = Pose()

                pose.position.x = goal_pose[0][0]
                pose.position.y = goal_pose[0][1]
                pose.position.z = goal_pose[0][2]

                q = Rotation.from_euler("xyz", goal_pose[1]).as_quat()
                pose.orientation.x = q[0]
                pose.orientation.y = q[1]
                pose.orientation.z = q[2]
                pose.orientation.w = q[3]

                if SHOW_RVIZ_MARKERS:
                    grasp_markers = get_grasp_marker(
                        header=Header(
                            stamp=self.get_clock().now().to_msg(),
                            frame_id="torso",
                        ),
                        grasp_pose=pose,
                        marker_id=marker_id * 2,
                        tip_length=0.1,  # GRASP_MARKER_TIP_LEN, taken from simple_grasp_pose.py
                        width=40,  # GRASP_MARKER_WIDTH, taken from simple_grasp_pose.py
                        score=1.0,
                        color=[0.0, 1.0, 0.0, 1.0],
                        lifetime=15.0,
                    )
                    self.marker_array.markers.extend(grasp_markers.markers)
                    if name == "l_arm":
                        self._marker_pub.publish(self.marker_array)

                stack.span.set_attributes(
                    {
                        "sol": sol.astype(float).tolist(),
                        "is_reachable": bool(is_reachable),
                        "state": state,
                        "control_ik.previous_sol[name]": self.control_ik.previous_sol[
                            name
                        ].tolist(),
                        "control_ik.last_call_t2[name]": str(
                            self.control_ik.last_call_t[name]
                        ),
                    }
                )

            else:
                self.logger.error("IK target pose should be only for the arms")
                raise ValueError("IK target pose should be only for the arms")

            msg = Float64MultiArray()
            msg.data = sol
            forward_publisher.publish(msg)
            reachability_msg = ReachabilityState()
            reachability_msg.header.stamp = self.get_clock().now().to_msg()
            reachability_msg.header.frame_id = "torso"
            reachability_msg.is_reachable = bool(is_reachable)
            reachability_msg.state = state
            reachability_msg.order_id = order_id
            self.reachability_pub[name].publish(reachability_msg)

    def on_target_pose(self, msg: PoseStamped, name, q0, forward_publisher):
        """
        # LEGACY (for old ros bags)
        """
        M = ros_pose_to_matrix(msg.pose)

        if "arm" in name:
            current_joints = self.get_current_position(self.chain[name])
            error, current_pose = forward_kinematics(
                self.fk_solver[name],
                current_joints,
                self.chain[name].getNrOfJoints(),
            )
            sol, is_reachable, state = self.control_ik.symbolic_inverse_kinematics(
                name,
                M,
                "continuous",
                current_joints=current_joints,
                constrained_mode="low_elbow",  # "unconstrained"
                current_pose=current_pose,
            )
        else:
            error, sol = inverse_kinematics(
                self.ik_solver[name],
                q0=q0,
                target_pose=M,
                nb_joints=self.chain[name].getNrOfJoints(),
            )
            sol = limit_orbita3d_joints(sol, self.orbita3D_max_angle)
            # TODO: check error

        # Limit the speed of the joints if needed
        # TODO FIXME disabled this "safety" because it can create very fast multiturns.
        # TODO divide the delta by the control period to manipulate speeds
        # current_position = np.array(self.get_current_position(self.chain[name]))
        # raw_vel = (np.array(sol) - current_position)
        # vel = np.clip(raw_vel, -self.max_joint_vel[name], self.max_joint_vel[name])
        # # save the max speed for the next iteration
        # if not np.allclose(raw_vel, vel):
        #     self.logger.warning(f"{name} Joint velocity limit reached. \nRaw vel: {raw_vel}\nClipped vel: {vel}")

        # smoothed_sol = current_position + vel
        # msg = Float64MultiArray()
        # msg.data = smoothed_sol.tolist()

        msg = Float64MultiArray()
        msg.data = sol
        forward_publisher.publish(msg)

    def on_averaged_target_pose(self, msg: PoseStamped, name, q0, forward_publisher):
        self.averaged_pose[name].append(msg.pose)
        avg_pose = self.averaged_pose[name].mean()

        M = ros_pose_to_matrix(avg_pose)
        if "arm" in name:
            current_joints = self.get_current_position(self.chain[name])
            error, current_pose = forward_kinematics(
                self.fk_solver[name],
                current_joints,
                self.chain[name].getNrOfJoints(),
            )
            sol, is_reachable, state = self.control_ik.symbolic_inverse_kinematics(
                name,
                M,
                "continuous",
                current_joints=current_joints,
                interval_limit=[-4 * np.pi / 5, 0],
                current_pose=current_pose,
            )
        else:
            error, sol = inverse_kinematics(
                self.ik_solver[name],
                q0=q0,
                target_pose=M,
                nb_joints=self.chain[name].getNrOfJoints(),
            )
            sol = limit_orbita3d_joints(sol, self.orbita3D_max_angle)

        # TODO: check error
        current_position = np.array(self.get_current_position(self.chain[name]))

        vel = np.array(sol) - current_position
        # vel = np.clip(vel, -self.max_joint_vel[name], self.max_joint_vel[name])

        smoothed_sol = current_position + vel

        msg = Float64MultiArray()
        msg.data = smoothed_sol.tolist()

        forward_publisher.publish(msg)

    def get_current_position(self, chain) -> List[float]:
        joints = self.get_chain_joints_name(chain)
        return [self._current_pos[j] for j in joints]

    def wait_for_joint_state(self):
        while not self.joint_state_ready.is_set():
            self.logger.info("Waiting for /joint_states...")
            rclpy.spin_once(self)

    def on_joint_state(self, msg: JointState):
        for j, pos in zip(msg.name, msg.position):
            self._current_pos[j] = pos

        self.joint_state_ready.set()

    def retrieve_urdf(self, timeout_sec: float = 15):
        self.logger.info('Retrieving URDF from "/robot_description"...')

        qos_profile = QoSProfile(depth=1)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.urdf = None

        def urdf_received(msg: String):
            self.urdf = msg.data

        self.create_subscription(
            msg_type=String,
            topic="/robot_description",
            qos_profile=qos_profile,
            callback=urdf_received,
        )
        rclpy.spin_once(self, timeout_sec=timeout_sec)

        if self.urdf is None:
            self.logger.error("Could not retrieve the URDF!")
            raise EnvironmentError("Could not retrieve the URDF!")

        self.logger.info("Done!")

        return self.urdf

    def check_position(self, js: JointState, chain) -> List[float]:
        pos = dict(zip(js.name, js.position))
        try:
            joints = [pos[j] for j in self.get_chain_joints_name(chain)]
            return joints
        except KeyError:
            self.logger.warning(
                f"Incorrect joints found ({js.name} vs {self.get_chain_joints_name(chain)})"
            )
            raise

    def get_chain_joints_name(self, chain):
        joints = []

        for i in range(chain.getNrOfSegments()):
            joint = chain.getSegment(i).getJoint()
            if joint.getType() == joint.RotAxis:
                joints.append(joint.getName())

        return joints


def main():
    rclpy.init()
    node = PollenKdlKinematics()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
