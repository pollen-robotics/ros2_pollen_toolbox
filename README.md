# Collection of ROS2 utility packages

## Packages

* [pollen_generic_description](./pollen_generic_description/) - URDF/ros2_control description of generic robot components
* [pid_command_controller](./pid_command_controller/) - ROS2 control PID command controller
* [pollen_msgs](./pollen_msgs/) - ROS2 messages for Pollen Robotics robots
* [dynamic_state_router](./dynamic_state_router/) - ROS2 node to simplify the use of forward controllers
* [pollen_kdl_kinematics](./pollen_kdl_kinematics/) - ROS2 bindings of KDL kinematics
* [pollen_goto](./pollen_goto/) - ROS2 Action server and client to perform gotos (joint interpolations)

See the readme of each package for more information.

## Installation

* Clone the repository into your ROS2 workspace
* Build the workspace with `colcon build --symlink-install`
