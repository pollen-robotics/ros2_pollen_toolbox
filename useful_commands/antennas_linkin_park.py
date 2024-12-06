import math
import time
import rclpy
from rclpy.node import Node
from control_msgs.msg import DynamicJointState, InterfaceValue


class SinusoidalJointCommandPublisher(Node):
    def __init__(self):
        super().__init__('sinusoidal_joint_command_publisher')
        self.publisher = self.create_publisher(
            DynamicJointState, '/dynamic_joint_commands', 10
        )

        self.amplitude = 1.0  # radians                                                                                                                                                                     
        self.joint_names = ['antenna_right', 'antenna_left']
        self.interface_names = ['position', 'position']
        self.publish_rate = 200  # Hz                                                                                                                                                                       
        self.bpm = 105.0
        self.bps = self.bpm/60.0
        self.frequency = self.bps/2.0  # Hz                                                                                                                                                                 

    def publish_sinusoidal_commands(self):
        """Publish sinusoidal commands to the joints."""
        start_time = time.time()+0.5

        while rclpy.ok():
            current_time = time.time() - start_time
            sinusoidal_value = self.amplitude * math.sin(2 * math.pi * self.frequency * current_time)

            # Create DynamicJointState message                                                                                                                                                              
            cmd_msg = DynamicJointState()
            cmd_msg.header.stamp = self.get_clock().now().to_msg()

            for joint, interface in zip(self.joint_names, self.interface_names):
                cmd_msg.joint_names.append(joint)
                inter = InterfaceValue()
                inter.interface_names.append(interface)
                inter.values.append(sinusoidal_value)
                cmd_msg.interface_values.append(inter)

            # Publish the message                                                                                                                                                                           
            self.publisher.publish(cmd_msg)
            self.get_logger().info(f"Published: {cmd_msg}")

            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(1.0/self.publish_rate)
            
def main(args=None):
    rclpy.init(args=args)
    node = SinusoidalJointCommandPublisher()

    try:
        print("OK")
        node.publish_sinusoidal_commands()
        print("OVER")
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
