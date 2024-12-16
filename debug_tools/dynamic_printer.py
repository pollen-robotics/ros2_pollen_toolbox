import fnmatch
import signal
from collections import defaultdict
from threading import Event, Lock, Thread

import rclpy
from control_msgs.msg import DynamicJointState
from rclpy.node import Node
from rclpy.time import Time

try:
    from rich import print
    from rich.console import Console
    from rich.live import Live
    from rich.table import Table
except ImportError:
    print('rich not available: pip install rich')
    exit(1)

SHOULD_EXIT = False


def handler(signum, frame):
    SHOULD_EXIT = True


signal.signal(signal.SIGINT, handler)


def partial_match(patterns, name):
    for pattern in patterns:
        if '*' in pattern:
            return True if len(fnmatch.filter([name], pattern))>0 else False
        else:
            return True if pattern == name else False



class DynamicState(Node):
    def __init__(self, name, joints, interfaces):
        super().__init__(node_name=name)
        self.logger = self.get_logger()
        self.joints=joints
        self.interfaces=interfaces

        self.joint_command = {}
        self.last_ts=None
        self.joint_state = {}
        self.joint_state_ready = Event()
        self.joint_state_sub = self.create_subscription(
            msg_type=DynamicJointState,
            topic="/dynamic_joint_states",
            qos_profile=5,
            callback=self.on_dynamic_joint_states,
        )
        # self.console = Console()

        self.table=None
        self.table_ready = Event()
    def on_dynamic_joint_states(self, msg: DynamicJointState):
        """Retreive the joint state from /dynamic_joint_states."""

        if not self.joint_state_ready.is_set():
            for uid, name in enumerate(msg.joint_names):
                self.joint_state[name] = {}
                self.joint_state[name]["name"] = name
                self.joint_state[name]["uid"] = uid

                self.joint_command[name] = {}

        for uid, (name, kv) in enumerate(
            zip(msg.joint_names, msg.interface_values)
        ):
            for k, v in zip(kv.interface_names, kv.values):
                if not "mimic" in name:
                    self.joint_state[name][k] = v

        if not self.joint_state_ready.is_set():
            for name, state in self.joint_state.items():
                if "position" in state:
                    state["target_position"] = state["position"]

            self.joint_state_ready.set()
        self.last_ts=Time.from_msg(msg.header.stamp)

        self.print_data()



    def print_data(self):
        if self.joint_state_ready.is_set():

            self.table=Table(title=f"dynamic_joint_state {self.last_ts.nanoseconds/1e9:.3f}s")

            self.table.add_column("Joint",justify="right", style="cyan", no_wrap=True)
            self.table.add_column("Interface", justify="right", style="magenta")
            self.table.add_column("Value", justify="left", style="green")


            for name, state in self.joint_state.items():
                first=True
                self.table.add_section()
                if self.joints is None or partial_match(self.joints, name):
                    for s,v in state.items():
                        if s!='name' and (self.interfaces is None or partial_match(self.interfaces,s)) :
                            if first:
                                self.table.add_row(name, s,str(v))
                                first=False
                            else:
                                self.table.add_row('', s,str(v))
            self.table_ready.set()

        return self.table
    def wait_for_setup(self):
        while not self.joint_state_ready.is_set():
            self.logger.info("Waiting for /dynamic_joint_states...")
            rclpy.spin_once(self)
        self.logger.info("Setup done!")



def main():
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('-j','--joints',nargs='+', help='list of joints to print (e.g. r_shoulder_pitch r_shoulder_roll). You can use wildcards (e.g "*shoudler*")')
    parser.add_argument('-i','--interfaces', nargs='+', help='list of interfaces to print (e.g. position velocity). You can use wildcards (e.g "*gain")')

    args = parser.parse_args()

    rclpy.init()
    node = DynamicState(
        name="dynamic_printer",
        joints=args.joints,
        interfaces=args.interfaces,
    )

    with Live(node.print_data(), refresh_per_second=5, vertical_overflow="visible" ) as live:
        node.wait_for_setup()
        while rclpy.ok() and not SHOULD_EXIT:
            rclpy.spin_once(node)
            live.update(node.print_data())


if __name__ == "__main__":
    main()
