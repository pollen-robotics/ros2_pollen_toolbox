import argparse
import asyncio
import logging
import sys
import time

import numpy as np
from aiortc import RTCDataChannel
from gst_signalling import GstSession, GstSignallingConsumer
from gst_signalling.utils import find_producer_peer_id_by_name
from reachy2_sdk_api.hand_pb2 import (
    HandPosition,
    HandPositionRequest,
    ParallelGripperPosition,
)
from reachy2_sdk_api.reachy_pb2 import ReachyState
from reachy2_sdk_api.webrtc_bridge_pb2 import (
    AnyCommand,
    AnyCommands,
    Connect,
    GetReachy,
    HandCommand,
    ArmCommand,
    ServiceRequest,
    ServiceResponse,
)
from google.protobuf.wrappers_pb2 import FloatValue
from reachy2_sdk_api.arm_pb2 import ArmCartesianGoal
from reachy2_sdk_api.kinematics_pb2 import Matrix4x4
import os
import time


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

            @pc.on("datachannel")  # type: ignore[misc]
            async def on_datachannel(channel: RTCDataChannel) -> None:
                self.logger.info(f"Joined new data channel: {channel.label}")

                if channel.label.startswith("reachy_state"):
                    self.handle_state_channel(channel)

                if channel.label.startswith("reachy_command"):
                    self.ensure_send_command(channel)

                if channel.label == "service":
                    await self.setup_connection(channel)

    async def run_consumer(self) -> None:
        await self.signaling.connect()
        await self.signaling.consume()

    async def close(self) -> None:
        await self.signaling.close()

    async def setup_connection(self, channel: RTCDataChannel) -> None:
        @channel.on("message")  # type: ignore[misc]
        def on_service_message(message: bytes) -> None:
            response = ServiceResponse()
            response.ParseFromString(message)

            if response.HasField("connection_status"):
                self.connection = response.connection_status
                self.connected.set()

            if response.HasField("error"):
                print(f"Received error message: {response.error}")

        # Ask for Reachy description (id, present parts, etc.)
        req = ServiceRequest(
            get_reachy=GetReachy(),
        )
        channel.send(req.SerializeToString())
        await self.connected.wait()
        self.logger.info(f"Got reachy: {self.connection.reachy}")

        # Then, Request for state stream update and start sending commands
        req = ServiceRequest(
            connect=Connect(
                reachy_id=self.connection.reachy.id,
                update_frequency=100,
            )
        )
        channel.send(req.SerializeToString())

    def handle_state_channel(self, channel: RTCDataChannel) -> None:
        @channel.on("message")  # type: ignore[misc]
        def on_message(message: bytes) -> None:
            reachy_state = ReachyState()
            reachy_state.ParseFromString(message)
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


    def get_arm_cartesian_goal(self, x: float, y: float, z: float) -> None:
        goal = np.array(
            [
                [0, 0, 1, x],
                [0, 1, 0, y],
                [1, 0, 0, z],
                [0, 0, 0, 1],
            ]
        )
        return ArmCartesianGoal(
            id={"id": 1, "name": "r_arm"},
            goal_pose=Matrix4x4(data=goal.flatten().tolist()),
            duration=FloatValue(value=1.0),
        )

    def ensure_send_command(self, channel: RTCDataChannel, freq: float = 100) -> None:
        async def send_command() -> None:
            print("coucou")
            radius = 0.5  # Circle radius
            fixed_x = 1  # Fixed x-coordinate
            center_y, center_z = 0, 0  # Center of the circle in y-z plane
            num_steps = 200  # Number of steps to complete the circle
            frequency = 500  # Update frequency in Hz
            step = 0  # Current step
            # with open(self.fifo_path, "w") as fifo:

            while True:
                angle = 2 * np.pi * (step / num_steps)
                step += 1
                if step >= num_steps:
                    step = 0
                # Calculate y and z coordinates
                y = center_y + radius * np.cos(angle)
                z = center_z + radius * np.sin(angle)
                commands = AnyCommands(
                    commands=[
                        AnyCommand(
                            arm_command=ArmCommand(arm_cartesian_goal=self.get_arm_cartesian_goal(fixed_x, y, z )),
                        ),
                    ],
                )
                channel.send(commands.SerializeToString())
                # timestamp = time.perf_counter_ns()
                # fifo.write(f"{timestamp}\n")

                await asyncio.sleep(1 / frequency)

        asyncio.ensure_future(send_command())


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
        logging.basicConfig(level=logging.INFO)

    sys.exit(main(args))
