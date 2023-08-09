# ROS2 PID Command Controller

Simple ROS2 control PID command controller. 

It supposes that your joint defines the following state and command interfaces:
* p_gain
* i_gain
* d_gain

See [joint.ros2_control.xacro](../pollen_generic_description/ros2_control/joint.ros2_control.xacro) for an example.

You can then set the gains using typical forward command publisher (e.g. with data such that: _[joint_1_p_gain, joint_1_i_gain, joint_1_d_gain, joint_2_p_gain, joint_2_i_gain, joint_2_d_gain, ...]_).