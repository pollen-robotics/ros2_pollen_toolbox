""" Easy access of any joint, sensor, gpio state/command interfaces.

High-level ROS interface on top of ROS2 control.
It lets you easily set a command interfaces for a single joint of a forward controller. 

Service:
- /get_dynamic_state (GetDynamicState) - retrieve any state(s) interface(s) for a specific joint/sensor/gpio (leave interfaces empty to get all)

Publication:
- /joint_commands (JointState) for each joints (match ros2 controllers frequency)

Subscription:
- /dynamic_joint_commands (DynamicJointState) - set any command(s) interface(s) for one or many joint/sensor/gpio


In more details, it:
- Subscribes to /dynamic_joint_states to get current values
- Publishes to corresponding /*_forward_controller (by automatically computing the diff)
- Specifically handles grippers commands (due to SafeGripperController)

"""
from collections import defaultdict
from functools import partial
from threading import Event, Lock, Thread

import rclpy
from rclpy.node import Node

from control_msgs.msg import DynamicJointState
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from pollen_msgs.msg import Gripper
from pollen_msgs.srv import GetDynamicState

import time
import numpy as np

from .forward_controller import ForwardControllersPool
import reachy2_monitoring as rm
import prometheus_client as pc

# Gripper OPEN/CLOSE position (in rads)
GRIPPER_OPEN_POSITION = np.deg2rad(130)
GRIPPER_CLOSE_POSITION = np.deg2rad(-5)

class TracedCommands:
    def __init__(self, traceparent="") -> None:
        self.traceparent = traceparent
        self.commands = defaultdict(dict)
        self.timestamp = 0

    def clear(self):
        self.traceparent = ""
        self.commands.clear()
        self.timestamp = 0



class DynamicStateRouterNode(Node):
    def __init__(self, node_name, controllers_file):
        super().__init__(node_name=node_name)
        self.logger = self.get_logger()
        self.tracer = rm.tracer(node_name)
        self.on_dynamic_joint_commands_counter = 0
        pc.start_http_server(10002)
        self.sum_joint_states = pc.Summary("dynamic_state_router_on_dynamic_joint_states_time",
                                           "Time spent during on_dynamic_joint_states callback")
        self.sum_joint_commands = pc.Summary("dynamic_state_router_on_dynamic_joint_commands_time",
                                             "Time spent during on_dynamic_joint_commands callback")
        self.sum_handle_commands = pc.Summary("dynamic_state_router_handle_commands_time",
                                              "Time spent during handle_commands callback")

        self.freq, self.forward_controllers = ForwardControllersPool.parse(
            self.logger, controllers_file
        )

        # Subscribe to /dynamic_joint_states
        self.joint_state_ready = Event()
        self.joint_state = {}
        self.joint_state_sub = self.create_subscription(
            msg_type=DynamicJointState,
            topic="/dynamic_joint_states",
            qos_profile=5,
            callback=self.on_dynamic_joint_states,
        )

        self.joint_command = {}

        # We wait to retrieve all setup info
        self.wait_for_setup()

        # Subscribe to each /forward_position_controller so we got target position updates
        self.forward_position_controller_sub = {
            fc.name: self.create_subscription(
                msg_type=Float64MultiArray,
                topic=f"/{fc.name}/commands",
                qos_profile=5,
                callback=partial(
                    self.on_forward_position_controller_update, controller_name=fc.name
                ),
            )
            for fc in self.forward_controllers.get_controllers_for_interface("position")
        }

        # Now we start our own service, publication and subscription

        # SERVICE: /get_dynamic_state (GetDynamicState)
        self.get_dyn_state_service = self.create_service(
            srv_type=GetDynamicState,
            srv_name="/get_dynamic_state",
            callback=self.get_dyn_state_cb,
        )

        # PUBLICATION: /joint_commands (JointState)
        self.joint_commands_pub = self.create_publisher(
            msg_type=JointState,
            topic="/joint_commands",
            qos_profile=5,
        )
        self.joint_commands_timer = self.create_timer(
            timer_period_sec=1 / self.freq,
            callback=self.publish_joint_commands,
        )

        # SUBSCRIPTION: /dynamic_joint_commands (DynamicJointState)
        # We need to get publisher to all forward controllers and the gripper safe controller
        self.joint_command_request_pub = Event()
        self.requested_commands = TracedCommands()
        self.pub_lock = Lock()
        self.publish_command_t = Thread(target=self.publish_command_loop)
        self.publish_command_t.daemon = True
        self.publish_command_t.start()

        self.fc_publisher = {
            fc.name: self.create_publisher(
                msg_type=Float64MultiArray,
                topic=f"/{fc.name}/commands",
                qos_profile=5,
            )
            for fc in self.forward_controllers.all()
        }
        self.logger.error(f"list of self.forward_controllers.all() : {self.forward_controllers.all()}")
        self.gripper_pub = self.create_publisher(
            msg_type=Gripper,
            topic="/grippers/commands",
            qos_profile=5,
        )

        self.joint_command_sub = self.create_subscription(
            msg_type=DynamicJointState,
            topic="/dynamic_joint_commands",
            qos_profile=5,
            callback=self.on_dynamic_joint_commands,
        )

        # We are ready to go! Log, current config
        self.logger.info("DynamicStateRouterNode setup:")
        for fc in self.forward_controllers.all():
            self.logger.info(f"\t- {fc.name}: {fc.joints} ({fc.interface})")
        self.logger.info("DynamicStateRouterNode ready!")

    # Service: GetDynamicState
    def get_dyn_state_cb(
        self, request: GetDynamicState.Request, response: GetDynamicState.Response
    ) -> GetDynamicState.Response:
        """Conveniant service to get interface values of a single joint/sensor/gpio."""
        if request.name not in self.joint_state.keys():
            self.logger.warning(
                f"Name should be one of {list(self.joint_state.keys())} (got '{request.name}' instead)"
            )
            return response

        response.name = request.name

        joint = self.joint_state[request.name]
        possible_interfaces = list(joint.keys())
        possible_interfaces.remove("name")

        if request.interfaces == []:
            for interface in possible_interfaces:
                response.interfaces.append(interface)
                response.values.append(joint[interface])
        else:
            for interface in request.interfaces:
                if interface not in joint:
                    self.logger.warning(
                        f"Interface should be one of {possible_interfaces} (got '{interface}' instead)"
                    )
                    continue

                response.interfaces.append(interface)
                response.values.append(joint[interface])

        return response

    # Subscription: dynamic_joint_commands (DynamicJointState)
    def on_dynamic_joint_commands(self, command: DynamicJointState):
        """Retreive the joint commands from /dynamic_joint_commands."""
        self.on_dynamic_joint_commands_counter += 1
        with rm.PollenSpan(tracer=self.tracer, trace_name="on_dynamic_joint_commands") as stack, self.sum_joint_commands.time():
            start_wait = time.time_ns()
            stack.span.set_attributes({
                    "on_dynamic_joint_commands_counter": self.on_dynamic_joint_commands_counter
                })
            with self.pub_lock:
                rm.travel_span("wait_for_pub_lock",
                               start_time=start_wait,
                               tracer=self.tracer,
                               )
                self.requested_commands.traceparent = rm.traceparent()
                for name, iv in zip(command.joint_names, command.interface_values):
                    if name not in self.joint_state:
                        self.logger.warning(
                            f'Unknown joint "{name}" ({list(self.joint_state.keys())})'
                        )
                        continue

                    for k, v in zip(iv.interface_names, iv.values):
                        if k not in self.joint_state[name]:
                            self.logger.warning(
                                f'Unknown interface for joint "{k}" ({list(self.joint_state[name].keys())})'
                            )
                            continue
                        self.requested_commands.commands[name][k] = v
            self.requested_commands.timestamp = time.time_ns()
            self.joint_command_request_pub.set()
        self.on_dynamic_joint_commands_counter -= 1

    def publish_command_loop(self):
        while rclpy.ok():
            self.joint_command_request_pub.wait()
            rm.travel_span("wait_for_command_request_pub",
                           start_time=self.requested_commands.timestamp,
                           tracer=self.tracer,
                           context=rm.ctx_from_traceparent(self.requested_commands.traceparent))

            with self.pub_lock:
                self.handle_commands(self.requested_commands)
                self.requested_commands.clear()
                self.joint_command_request_pub.clear()

    def publish_joint_commands(self):
        msg = JointState()

        for j in self.joint_state.values():
            if "target_position" in j:
                msg.name.append(j["name"])
                msg.position.append(j["target_position"])

        self.joint_commands_pub.publish(msg)

    def handle_commands(self, commands):
        ctx = rm.ctx_from_traceparent(commands.traceparent)
        with rm.PollenSpan(tracer=self.tracer,
                                       trace_name="handle_commands",
                                       context=ctx), self.sum_handle_commands.time():
            gripper_commands = TracedCommands(traceparent=rm.traceparent())
            regular_commands = TracedCommands(traceparent=rm.traceparent())
            pid_commands = TracedCommands(traceparent=rm.traceparent())

            for joint, iv in commands.commands.items():
                for interface, value in iv.items():
                    if joint.endswith("finger") and interface == "position":
                        open_pos = GRIPPER_OPEN_POSITION
                        close_pos = GRIPPER_CLOSE_POSITION
                        goal_pos = close_pos + value * (open_pos - close_pos)
                        if open_pos < close_pos:
                            goal_pos = np.clip(goal_pos, open_pos, close_pos)
                        else:
                            goal_pos = np.clip(goal_pos, close_pos, open_pos)
                        regular_commands.commands[joint].update({interface: goal_pos})
                    elif interface in ("p_gain", "i_gain", "d_gain"):
                        pid_commands.commands[joint].update({interface: value})
                    else:
                        regular_commands.commands[joint].update({interface: value})
            # if gripper_commands:
            #     self.handle_gripper_commands(gripper_commands)
            if pid_commands:
                self.handle_pid_commands(pid_commands)
            if regular_commands:
                self.handle_regular_commands(regular_commands)

    def handle_pid_commands(self, commands):
        # TODO implement PollenSpan with TracedCommands
        return

        pid_fc = self.forward_controllers["forward_pid_controller"]
        msg = Float64MultiArray()

        for j in pid_fc.joints:
            for gain in ("p_gain", "i_gain", "d_gain"):
                if j in commands.commands and gain in commands.commands[j]:
                    msg.data.append(commands.commands[j][gain])
                elif j in self.joint_command and gain in self.joint_command[j]:
                    msg.data.append(self.joint_command[j][gain])
                else:
                    msg.data.append(self.joint_state[j][gain])

        for j, gains in commands.commands.items():
            for g, val in gains.items():
                self.joint_command[j][g] = val

        self.fc_publisher["forward_pid_controller"].publish(msg)

    def handle_regular_commands(self, commands):
        ctx = rm.ctx_from_traceparent(commands.traceparent)
        with rm.PollenSpan(tracer=self.tracer,
                                       trace_name="handle_regular_commands",
                                       context=ctx):
            # Group commands by forward controller
            to_pub = defaultdict(dict)
            for joint, iv in commands.commands.items():
                for interface, value in iv.items():
                    try:
                        fc = self.forward_controllers.get_corresponding_controller(
                            joint, interface
                        )
                    except KeyError:
                        self.logger.error(
                            f"Unknown interface '{interface}' for joint '{joint}'"
                        )
                        continue
                    to_pub[fc][joint] = value

            for fc, new_cmd in to_pub.items():
                msg = Float64MultiArray()

                pub_interface = (
                    fc.interface if fc.interface != "position" else "target_position"
                )

                msg.data = []
                for j in fc.joints:
                    if j in new_cmd:
                        msg.data.append(new_cmd[j])
                    elif pub_interface in self.joint_command[j]:
                        msg.data.append(self.joint_command[j][pub_interface])
                    else:
                        msg.data.append(self.joint_state[j][pub_interface])

                for j, val in new_cmd.items():
                    self.joint_command[j][pub_interface] = val
                self.fc_publisher[fc.name].publish(msg)

    # Internal ROS2 subscription
    # Subscription: DynamicJointState cb
    def on_dynamic_joint_states(self, state: DynamicJointState):
        """Retreive the joint state from /dynamic_joint_states."""
        with self.sum_joint_states.time():
            if not self.joint_state_ready.is_set():
                for uid, name in enumerate(state.joint_names):
                    self.joint_state[name] = {}
                    self.joint_state[name]["name"] = name
                    self.joint_state[name]["uid"] = uid

                    self.joint_command[name] = {}

            for uid, (name, kv) in enumerate(
                zip(state.joint_names, state.interface_values)
            ):
                for k, v in zip(kv.interface_names, kv.values):
                    if not "mimic" in name:
                        self.joint_state[name][k] = v

            if not self.joint_state_ready.is_set():
                for name, state in self.joint_state.items():
                    if "position" in state:
                        state["target_position"] = state["position"]

                self.joint_state_ready.set()

    def on_forward_position_controller_update(
        self, msg: Float64MultiArray, controller_name: str
    ):
        fc = self.forward_controllers[controller_name]

        for j, v in zip(fc.joints, msg.data):
            self.joint_state[j]["target_position"] = v

    def wait_for_setup(self):
        while not self.joint_state_ready.is_set():
            self.logger.info("Waiting for /dynamic_joint_states...")
            rclpy.spin_once(self)
        self.logger.info("Setup done!")


def main():
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--ros-args", action="store_true")
    parser.add_argument("controllers_file")
    args = parser.parse_args()

    rclpy.init()
    node = DynamicStateRouterNode(
        node_name="dynanic_state_router_node",
        controllers_file=args.controllers_file,
    )
    rclpy.spin(node)


if __name__ == "__main__":
    main()
