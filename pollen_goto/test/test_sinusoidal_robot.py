import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from pollen_msgs.action import Goto
from pollen_goto.interpolation import JointSpaceInterpolationMode

class SinusoidalTest(Node):
    def __init__(self):
        super().__init__('sinusoidal_test')
        self._action_client = ActionClient(self, Goto, 'r_arm_goto')
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Action server found!')

    def send_goal(self):
        goal_msg = Goto.Goal()
        goal_msg.request.interpolation_mode = JointSpaceInterpolationMode.SINUSOIDAL_FUNC
        goal_msg.request.duration = 2.0  # 2 seconds
        goal_msg.request.joints = ['r_shoulder_pitch', 'r_shoulder_roll']
        goal_msg.request.positions = [0.5, 0.3]  # Target positions

        self.get_logger().info('Sending goal request...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    test_node = SinusoidalTest()
    test_node.send_goal()
    rclpy.spin(test_node)

if __name__ == '__main__':
    main() 