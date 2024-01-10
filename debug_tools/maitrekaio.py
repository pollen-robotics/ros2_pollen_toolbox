import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from collections import deque
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class PoseListener(Node):
    def __init__(self):
        super().__init__('pose_listener')

        # High frequency QoS profile
        high_freq_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Prioritizes speed over guaranteed delivery
            history=HistoryPolicy.KEEP_LAST,            # Keeps only a fixed number of messages
            depth=1,                                    # Minimal depth, for the latest message
            # Other QoS settings can be adjusted as needed
        )

        self.subscription = self.create_subscription(
            PoseStamped,
            '/r_arm/target_pose',
            self.listener_callback,
            high_freq_qos_profile
        )
        self.subscription  # prevent unused variable warning
        self.delays = deque(maxlen=1000)  # Deque for storing delays

    def listener_callback(self, msg):
        # print(f"Received PoseStamped message with timestamp: {msg.header.stamp}")
        now = self.get_clock().now().to_msg()
        # print(f"Current time: {now}")
        msg_time = rclpy.time.Time.from_msg(msg.header.stamp)
        now = rclpy.time.Time.from_msg(now)
        # print(f"Message time: {msg_time}")
        # print(f"Current newtime: {now}")
        delay = now - msg_time
        # delay_msec = delay.nanoseconds / 1e6
        # print delay in eevery format, seconds,  millisecond , micro second and nano sceond

        # self.get_logger().info(f': {delay.nanoseconds / 1e9} s\t{delay.nanoseconds / 1e6} ms\t {delay.nanoseconds / 1e3} us\t {delay.nanoseconds} ns')
        # self.get_logger().info(f': {delay.nanoseconds / 1e6} ms')

        delay_ms = delay.nanoseconds / 1e6

        # Update deque with new delay
        self.delays.append(delay_ms)

        # Calculate mean delay
        mean_delay = sum(self.delays) / len(self.delays)

        self.get_logger().info(f'Delay: {delay_ms} ms, Mean Delay: {mean_delay} ms over last 1000 measurements')


def main(args=None):
    rclpy.init(args=args)
    pose_listener = PoseListener()
    rclpy.spin(pose_listener)
    pose_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
