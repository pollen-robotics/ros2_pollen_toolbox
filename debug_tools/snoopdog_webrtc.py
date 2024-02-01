import argparse
import asyncio
import logging
import os
import sys
import time

import gi
import numpy as np

gi.require_version("Gst", "1.0")
from gi.repository import GLib, Gst
from google.protobuf.wrappers_pb2 import FloatValue
from gst_signalling import GstSignallingConsumer
from gst_signalling.gst_abstract_role import GstSession
from gst_signalling.utils import find_producer_peer_id_by_name
from reachy2_sdk_api.arm_pb2 import ArmCartesianGoal
from reachy2_sdk_api.hand_pb2 import (
    HandPosition,
    HandPositionRequest,
    ParallelGripperPosition,
)
from reachy2_sdk_api.kinematics_pb2 import Matrix4x4
from reachy2_sdk_api.reachy_pb2 import ReachyState
from reachy2_sdk_api.webrtc_bridge_pb2 import (
    AnyCommand,
    AnyCommands,
    ArmCommand,
    Connect,
    GetReachy,
    HandCommand,
    ServiceRequest,
    ServiceResponse,
)


class TeleopApp:
    def __init__(self, args: argparse.Namespace) -> None:
        self.logger = logging.getLogger(__name__)

        producer_peer_id = find_producer_peer_id_by_name(
            args.webrtc_signaling_host,
            args.webrtc_signaling_port,
            args.webrtc_producer_name,
        )

        self.signaling = GstSignallingConsumer(
            host=args.webrtc_signaling_host,
            port=args.webrtc_signaling_port,
            producer_peer_id=producer_peer_id,
        )

        self.connected = asyncio.Event()

        #
        # self.fifo_path = "/tmp/latency_fifo"
        #
        # # Create a FIFO if it doesn't exist
        # if not os.path.exists(self.fifo_path):
        #     os.mkfifo(self.fifo_path)

        @self.signaling.on("new_session")  # type: ignore[misc]
        def on_new_session(session: GstSession) -> None:
            self.logger.info(f"New session: {session}")

            pc = session.pc

            pc.connect("on-data-channel", self.on_data_channel_callback)

            """
            @pc.on("datachannel")  # type: ignore[misc]
            async def on_datachannel(channel: RTCDataChannel) -> None:
                self.logger.info(f"Joined new data channel: {channel.label}")

                if channel.label.startswith("reachy_state"):
                    self.handle_state_channel(channel)

                if channel.label.startswith("reachy_command"):
                    self.ensure_send_command(channel)

                if channel.label == "service":
                    await self.setup_connection(channel)
            """

    def on_data_channel_callback(self, webrtc: Gst.Element, data_channel: Gst.Element) -> None:  # type: ignore[no-untyped-def]
        self.logger.info(
            f'Joined new data channel: {data_channel.get_property("label")}'
        )
        if data_channel.get_property("label").startswith("reachy_state"):
            data_channel.connect("on-message-data", self.handle_state_channel)
        elif data_channel.get_property("label").startswith("reachy_command"):
            asyncio.to_thread(self.ensure_send_command(data_channel))
        elif data_channel.get_property("label") == "service":
            data_channel.connect("on-message-data", self.on_service_message)
            self.setup_connection(data_channel)

    async def run_consumer(self) -> None:
        await self.signaling.connect()
        await self.signaling.consume()

    async def close(self) -> None:
        await self.signaling.close()

    def on_service_message(self, data_channel, data: GLib.Bytes) -> None:  # type: ignore[no-untyped-def]
        response = ServiceResponse()
        response.ParseFromString(data.get_data())

        if response.HasField("connection_status"):
            self.connection = response.connection_status
            req = ServiceRequest(
                connect=Connect(
                    reachy_id=self.connection.reachy.id,
                    update_frequency=100,
                )
            )

            byte_data = req.SerializeToString()
            gbyte_data = GLib.Bytes.new(byte_data)
            data_channel.send_data(gbyte_data)

        if response.HasField("error"):
            self.logger.error(f"Received error message: {response.error}")

    def setup_connection(self, data_channel) -> None:
        req = ServiceRequest(
            get_reachy=GetReachy(),
        )
        byte_data = req.SerializeToString()
        gbyte_data = GLib.Bytes.new(byte_data)
        data_channel.send_data(gbyte_data)

    def handle_state_channel(self, data_channel, data: GLib.Bytes) -> None:  # type: ignore[no-untyped-def]
        reachy_state = ReachyState()
        reachy_state.ParseFromString(data.get_data())
        self.reachy_state = reachy_state

    # def ensure_send_command(self, channel: RTCDataChannel, freq: float = 100) -> None:
    #     async def send_command() -> None:
    #         while True:
    #             target = 0.5 - 0.5 * np.sin(2 * np.pi * 1 * time.time())
    #
    #             commands = AnyCommands(
    #                 commands=[
    #                     AnyCommand(
    #                         hand_command=HandCommand(
    #                             hand_goal=HandPositionRequest(
    #                                 id=self.connection.reachy.r_hand.part_id,
    #                                 position=HandPosition(parallel_gripper=ParallelGripperPosition(position=target)),
    #                             ),
    #                         ),
    #                     ),
    #                 ],
    #             )
    #             channel.send(commands.SerializeToString())
    #
    #             await asyncio.sleep(1 / freq)
    #
    #     asyncio.ensure_future(send_command())

    def get_arm_cartesian_goal(self, x: float, y: float, z: float, partid=1) -> None:
        goal = np.array(
            [
                [0, 0, 1, x],
                [0, 1, 0, y],
                [1, 0, 0, z],
                [0, 0, 0, 1],
            ]
        )
        return ArmCartesianGoal(
            id={"id": partid, "name": "r_arm"},
            goal_pose=Matrix4x4(data=goal.flatten().tolist()),
            duration=FloatValue(value=1.0),
        )

    def ensure_send_command(self, data_channel, freq: float = 100) -> None:
        radius = 0.5  # Circle radius
        fixed_x = 1  # Fixed x-coordinate
        center_y, center_z = 0, 0  # Center of the circle in y-z plane
        num_steps = 200  # Number of steps to complete the circle
        frequency = 1000  # Update frequency in Hz
        step = 0  # Current step
        circle_period = 3
        # with open(self.fifo_path, "w") as fifo:
        t0 = time.time()
        last_freq_counter = 0
        init = False
        freq_rates = []
        last_freq_update = time.time()

        while True:
            angle = 2 * np.pi * (step / num_steps)
            angle = 2 * np.pi * (time.time() - t0) / circle_period
            # print(angle)
            step += 1
            if step >= num_steps:
                step = 0
            # Calculate y and z coordinates
            y = center_y + radius * np.cos(angle)
            z = center_z + radius * np.sin(angle)
            commands = AnyCommands(
                commands=[
                    AnyCommand(
                        arm_command=ArmCommand(
                            arm_cartesian_goal=self.get_arm_cartesian_goal(
                                fixed_x, y, z
                            )
                        ),
                    ),
                ],
            )
            byte_data = commands.SerializeToString()
            gbyte_data = GLib.Bytes.new(byte_data)
            data_channel.send_data(gbyte_data)

            last_freq_counter += 1
            now = time.time()
            if now - last_freq_update > 1:
                current_freq_rate = int(last_freq_counter / (now - last_freq_update))

                self.logger.info(f"Freq {current_freq_rate} Hz")

                if init:
                    freq_rates.append(current_freq_rate)
                    if len(freq_rates) > 10000:
                        freq_rates.pop(0)
                    mean_freq_rate = sum(freq_rates) / len(freq_rates)
                    self.logger.info(f"[MEAN] Freq {mean_freq_rate} Hz")
                else:
                    init = True
                # Calculate mean values

                last_freq_counter = 0
                last_freq_update = now

            # await asyncio.sleep(1.0 / frequency)
            time.sleep(1.0 / frequency)

    """
    def ensure_send_command(self, channel: RTCDataChannel, freq: float = 100) -> None:
        async def send_command() -> None:
            print("coucou")
            radius = 0.5  # Circle radius
            fixed_x = 1  # Fixed x-coordinate
            center_y, center_z = 0, 0  # Center of the circle in y-z plane
            num_steps = 200  # Number of steps to complete the circle
            frequency = 10000  # Update frequency in Hz
            step = 0  # Current step
            circle_period = 3
            # with open(self.fifo_path, "w") as fifo:
            t0 = time.time()
            while True:
                angle = 2 * np.pi * (step / num_steps)
                angle = 2 * np.pi * (time.time() - t0) / circle_period
                print(angle)
                step += 1
                if step >= num_steps:
                    step = 0
                # Calculate y and z coordinates
                y = center_y + radius * np.cos(angle)
                z = center_z + radius * np.sin(angle)
                commands = AnyCommands(
                    commands=[
                        AnyCommand(
                            arm_command=ArmCommand(
                                arm_cartesian_goal=self.get_arm_cartesian_goal(
                                    fixed_x, y, z
                                )
                            ),
                        ),
                    ],
                )
                channel.send(commands.SerializeToString())

                # commands = AnyCommands(
                #     commands=[
                #         AnyCommand(
                #             arm_command=ArmCommand(arm_cartesian_goal=self.get_arm_cartesian_goal(fixed_x, y, z, partid=2 )),
                #         ),
                #     ],
                # )
                # channel.send(commands.SerializeToString())

                # target = 0.5 - 0.5 * np.sin(2 * np.pi * 1 * time.time())
                #
                # commands = AnyCommands(
                #     commands=[
                #         AnyCommand(
                #             hand_command=HandCommand(
                #                 hand_goal=HandPositionRequest(
                #                     id=self.connection.reachy.r_hand.part_id,
                #                     position=HandPosition(parallel_gripper=ParallelGripperPosition(position=target)),
                #                 ),
                #             ),
                #         ),
                #     ],
                # )
                # channel.send(commands.SerializeToString())

                # timestamp = time.perf_counter_ns()
                # fifo.write(f"{timestamp}\n")

                await asyncio.sleep(1 / frequency)

        asyncio.ensure_future(send_command())
    """


def main(args: argparse.Namespace) -> int:  # noqa: C901
    teleop = TeleopApp(args)

    # run event loop
    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(teleop.run_consumer())
    except KeyboardInterrupt:
        pass
    finally:
        loop.run_until_complete(teleop.close())

    return 0


if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    # WebRTC
    parser.add_argument(
        "--webrtc-signaling-host",
        type=str,
        default="127.0.0.1",
        help="Host of the gstreamer webrtc signaling server.",
    )
    parser.add_argument(
        "--webrtc-signaling-port",
        type=int,
        default=8443,
        help="Port of the gstreamer webrtc signaling server.",
    )
    parser.add_argument(
        "--webrtc-producer-name",
        type=str,
        default="grpc_webrtc_bridge",
        help="Name of the producer.",
    )

    # Logging
    parser.add_argument(
        "--verbose",
        "-v",
        action="store_true",
        help="Enable verbose logging.",
    )
    args = parser.parse_args()

    if args.verbose:
        logging.basicConfig(level=logging.DEBUG)
        os.environ["GST_DEBUG"] = "3"

    sys.exit(main(args))
