# Dynamic State Router

This ROS2 node intends to simplify the use of forward controllers used in ROS2 control.
The idea is to let you retrieve a specific interface from a joint directly by its name. Similarly, you can set a specific interface for a joint directly.

To do that, this node uses the controller file as a parameter.

In more details, this node exposes:

A new service:
- /get_dynamic_state (pollen_msgs/GetDynamicState) - retrieve any state(s) interface(s) for a specific joint/sensor/gpio

A new subscription to the topic:
- /dynamic_joint_commands (control_msgs/DynamicJointState) - set any command(s) interface(s) for one or many joint/sensor/gpio
