import copy
import time
from functools import partial
from threading import Event
from typing import List

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from pollen_msgs.srv import GetForwardKinematics, GetInverseKinematics
from pollen_msgs.msg import IKRequest, ReachabilityState
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.qos import (HistoryPolicy, QoSDurabilityPolicy, QoSProfile,
                       ReliabilityPolicy)
from reachy2_symbolic_ik.control_ik import ControlIK
from reachy2_symbolic_ik.symbolic_ik import SymbolicIK
from reachy2_symbolic_ik.utils import (allow_multiturn,
                                       get_best_continuous_theta,
                                       get_best_continuous_theta2,
                                       get_best_discrete_theta,
                                       get_euler_from_homogeneous_matrix,
                                       limit_orbita3d_joints,
                                       limit_orbita3d_joints_wrist,
                                       limit_theta_to_interval,
                                       get_best_theta_to_current_joints,)
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Header, String
from visualization_msgs.msg import MarkerArray

from pollen_grasping_utils.utils import get_grasp_marker

from .kdl_kinematics import (forward_kinematics, generate_solver,
                             inverse_kinematics, ros_pose_to_matrix)
from .pose_averager import PoseAverager

from . import tracing_helper
from opentelemetry import trace

class PollenKdlKinematics(LifecycleNode):
    def __init__(self):
        super().__init__("pollen_kdl_kinematics_node")
        self.logger = self.get_logger()

        self.urdf = self.retrieve_urdf()

        self.tracer = tracing_helper.tracer("pollen_kdl_kinematics_node")

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

        # High frequency QoS profile
        high_freq_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Prioritizes speed over guaranteed delivery
            history=HistoryPolicy.KEEP_LAST,  # Keeps only a fixed number of messages
            depth=1,  # Minimal depth, for the latest message
            # Other QoS settings can be adjusted as needed
        )

        self.orbita3D_max_angle = np.deg2rad(42.5)  # 43.5 is too much
        # Symbolic IK inits.
        # A balanced position between elbow down and elbow at 90°
        # self.prefered_theta = -4 * np.pi / 6  # 5 * np.pi / 4  # np.pi / 4
        # self.nb_search_points = 20
        # self.previous_theta = {}
        # self.previous_sol = {}

        for prefix in ("l", "r"):
            arm = f"{prefix}_arm"

            chain, fk_solver, ik_solver = generate_solver(self.urdf, "torso", f"{prefix}_arm_tip")

            # self.symbolic_ik_solver[arm] = SymbolicIK(
            #     arm=arm,
            #     upper_arm_size=0.28,
            #     forearm_size=0.28,
            #     gripper_size=0.10,
            #     wrist_limit=np.rad2deg(self.orbita3D_max_angle),
            #     # This is the "correct" stuff for alpha
            #     # shoulder_orientation_offset=[10, 0, 15],
            #     # elbow_orientation_offset=[0, 0, 0],
            #     # This is the "wrong" values currently used by the alpha
            #     # shoulder_orientation_offset=[0, 0, 15],
            #     # shoulder_position=[-0.0479, -0.1913, 0.025],
            # )

            # self.previous_theta[arm] = None
            # self.previous_sol[arm] = None
            # self.last_call_t[arm] = 0

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
                self.logger.info(f'Adding subscription on "{self.target_sub[arm].topic}"...')

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
                self.logger.info(f'Adding subscription on "{self.ik_target_sub[arm].topic}"...')

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
                self.default_max_joint_vel = 0.6  # Angular difference between current joint and requested joint.
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
                self.logger.info(f'Adding subscription on "{self.target_sub[arm].topic}"...')

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
        self.control_ik = ControlIK(
            logger=self.logger,
            current_joints=current_joints,
            current_pose=current_pose,
        )

        self.logger.info(f"Kinematics node ready!")
        self.trigger_configure()

        self._marker_pub = self.create_publisher(MarkerArray, "markers_grasp_triplet", 10)

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        # Dummy state to minimize impact on current behavior
        self.logger.info("Configuring state has been called, going into inactive to release event trigger")
        return TransitionCallbackReturn.SUCCESS

    def forward_kinematics_srv(
        self,
        request: GetForwardKinematics.Request,
        response: GetForwardKinematics.Response,
        name,
    ) -> GetForwardKinematics.Response:
        try:
            joint_position = self.check_position(request.joint_position, self.chain[name])
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
                current_pose=current_pose
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

    def on_ik_target_pose(self, msg: IKRequest, name, forward_publisher):

        self.logger.warn(f"on_target_pose.{name}: {msg.traceparent}")

        ctx = tracing_helper.ctx_from_traceparent(msg.traceparent)

        with self.tracer.start_as_current_span(f"{name}::on_target_pose",
                                                kind=trace.SpanKind.SERVER,
                                                context=ctx) as span:
            span.set_attributes({"rpc.system": "ros_topic", "server.address": "localhost"})

            M = ros_pose_to_matrix(msg.pose.pose)
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
                    constrained_mode=constrained_mode,
                    current_pose=current_pose,
                    d_theta_max=d_theta_max,
                    preferred_theta=preferred_theta,
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
            reachability_msg.is_reachable = is_reachable
            reachability_msg.state = state
            reachability_msg.order_id = order_id
            self.reachability_pub[name].publish(reachability_msg)


    def on_target_pose(self, msg: PoseStamped, name, q0, forward_publisher):
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
                current_pose=current_pose
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
                current_pose=current_pose
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
            self.logger.warning(f"Incorrect joints found ({js.name} vs {self.get_chain_joints_name(chain)})")
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
