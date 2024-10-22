import rclpy
import zmq
from functools import partial
import ruckig
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Header, String

NODE_NAME = "pollen_kdl_kinematics_node"
import time
import reachy2_monitoring as rm
import prometheus_client as prc
import pinocchio as pin
import numpy as np
from . import qp_utils as qu
from rclpy.lifecycle import LifecycleNode

from rclpy.qos import HistoryPolicy, QoSDurabilityPolicy, QoSProfile, ReliabilityPolicy
from copy import copy

from ruckig import InputParameter, OutputParameter, Result, Ruckig


JOINTS = [
    "_shoulder_pitch",
    "_shoulder_roll",
    "_elbow_yaw",
    "_elbow_pitch",
    "_wrist_roll",
    "_wrist_pitch",
    "_wrist_yaw",
]
LEFT_JOINTS = ["l" + x for x in JOINTS]
RIGHT_JOINTS = ["r" + x for x in JOINTS]

np.set_printoptions(formatter={"float": lambda x: "{0:0.2f}".format(x)})


class PollenControlNode(LifecycleNode):
    def __init__(self):
        super().__init__(NODE_NAME)
        prc.start_http_server(10004)

        self.T = 0.002  # [s]
        self.timer = self.create_timer(self.T, self.node_callback)

        self.dofs = 7
        self.inp = {}
        self.otg = {}
        self.out = {}
        self.forward_position_pub = {}

        self.joints_target = {}
        self.joints = {}
        self._current_pos = {}
        self.joints_target_inited = {}

        def zero_dofs():
            return [0.0] * self.dofs

        for arm in ["l_arm", "r_arm"]:
            self.joints_target_inited[arm] = False
            self.joints_target[arm] = zero_dofs()
            self.joints[arm] = zero_dofs()

            self.forward_position_pub[arm] = self.create_publisher(
                msg_type=Float64MultiArray,
                topic=f"/{arm}_forward_position_controller/commands",
                qos_profile=5,
            )

            self.otg[arm] = Ruckig(self.dofs, self.T)  # DoFs, control cycle
            self.inp[arm] = InputParameter(self.dofs)
            self.out[arm] = OutputParameter(self.dofs)


            # Set input parameters
            self.inp[arm].current_position = zero_dofs()
            self.inp[arm].current_velocity = zero_dofs()
            self.inp[arm].current_acceleration = zero_dofs()

            self.inp[arm].target_position = zero_dofs()
            self.inp[arm].target_velocity = zero_dofs()
            self.inp[arm].target_acceleration = zero_dofs()

            self.inp[arm].max_velocity = [5] * self.dofs
            self.inp[arm].max_acceleration = [10] * self.dofs
            self.inp[arm].max_jerk = [100] * self.dofs

        self.last_call = time.time_ns() / 1e9

        # self.zmq_context = zmq.Context()
        # self.zmq_sockets = {}
        # port = 555
        # self.poller = zmq.Poller()
        # for k, arm in enumerate(["left", "right"]):
        #     self.zmq_sockets[arm] = self.zmq_context.socket(zmq.SUB)
        #     self.zmq_sockets[arm].connect("tcp://localhost:%s" % (port + k))
        #     # self.zmq_sockets[arm].setsockopt(zmq.SUBSCRIBE, "9")
        #     self.zmq_sockets[arm].subscribe("")
        #     self.poller.register(self.zmq_sockets[arm], zmq.POLLIN)

        self.joint_state_sub = self.create_subscription(
            msg_type=JointState,
            topic="/joint_states",
            qos_profile=5,
            callback=self.on_joint_state,
        )

        self.joint_sub = {}
        # High frequency QoS profile
        high_freq_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Prioritizes speed over guaranteed delivery
            history=HistoryPolicy.KEEP_LAST,  # Keeps only a fixed number of messages
            depth=1,  # Minimal depth, for the latest message
            # Other QoS settings can be adjusted as needed
        )
        for arm in ["l_arm", "r_arm"]:
            self.joint_sub[arm] = self.create_subscription(
                msg_type=Float64MultiArray,
                topic=f"/target_joints_{arm}",
                qos_profile=high_freq_qos_profile,
                callback=partial(
                    self.on_joint_target,
                    name=arm,
                ),
            )

    def on_joint_target(self, msg: Float64MultiArray, name):
        self.joints_target[name] = msg.data
        print(f"{name}: {msg.data}")
        self.joints_target_inited[name] = True
        # for arm in ["l_arm", "r_arm"]:
        #     self.inp[arm].target_position = self.joints_target[arm]

    def on_joint_state(self, msg: JointState):
        for j, pos in zip(msg.name, msg.position):
            self._current_pos[j] = pos

        self.joints["l_arm"] = [self._current_pos[j] for j in LEFT_JOINTS]
        self.joints["r_arm"] = [self._current_pos[j] for j in RIGHT_JOINTS]
        # for arm in ["l_arm", "r_arm"]:
        #     self.inp[arm].current_position = self.joints[arm]

    def node_callback(self):
        now = time.time_ns() / 1e9
        dt_ms = (now - self.last_call) * 1000
        if dt_ms > (self.T * 1000) * 1.2:
            print(
                f"PollenControlNode: SLOWDOWN detected dt: {dt_ms:.2f}ms (should be {self.T*1000:.2f}ms)"
            )

        # socks = dict(self.poller.poll())
        # for arm, socket in self.zmq_sockets.items():
        #     if socket in socks and socks[socket] == zmq.POLLIN:
        #         msg = socket.recv()
        #         print(f"recv {arm}: {msg}")

        for arm in ["l_arm", "r_arm"]:
            msg = Float64MultiArray()
            msg.data = self.joints[arm]
            if not np.allclose(self.joints[arm], self.joints_target[arm],
                               atol=1e-03) and self.joints_target_inited[arm]:
                self.inp[arm].current_position = self.joints[arm]
                self.inp[arm].target_position = self.joints_target[arm]
                res = self.otg[arm].update(self.inp[arm], self.out[arm])
                print(f"{arm} ({self.out[arm].time:0.3f}): {self.out[arm].new_position}")
                self.out[arm].pass_to_input(self.inp[arm])
                msg.data = self.out[arm].new_position
            self.forward_position_pub[arm].publish(msg)

        self.last_call = now


def main():
    rclpy.init()
    node = PollenControlNode()
    rclpy.spin(node)
