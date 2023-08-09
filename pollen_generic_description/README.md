# Generic tools for ROS2 description packages

### URDF

* [inertial primitives](./urdf/inertial_primitives.urdf.xacro) - basic inertial primitives used to build simple URDF models (sphere, cylinder, cube)

### ROS2 control

* [basic joint](./ros2_control/joint.ros2_control.xacro) - basic single dof joint description for ROS2 control (state interface: position, velocity, effort, temperature, torque, speed_limit, torque_limit, p_gain, i_gain, d_gain; command interface: position, torque, speed_limit, torque_limit, p_gain, i_gain, d_gain)
