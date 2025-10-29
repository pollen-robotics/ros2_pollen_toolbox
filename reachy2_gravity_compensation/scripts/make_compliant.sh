ros2 topic pub --once /forward_torque_controller/commands std_msgs/Float64MultiArray "{ data:[1,1,1,1,1,1,1,1,1,1,1]}"
ros2 topic pub --once /forward_torque_limit_controller/commands std_msgs/Float64MultiArray "{data:[0.000, 0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000]}"
