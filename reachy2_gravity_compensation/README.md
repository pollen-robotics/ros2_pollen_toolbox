# This node provides gravity compensation for the Reachy2 robot using ROS2 control framework.

It subscribes to the robot's joint states and computes the necessary torques to counteract gravity, allowing for easier manipulation and interaction with the robot.
By deafult, it is launched in the background by the ros launch and is always on. 

To make the robot go to the gravity compensation mode, you will have to:
- enable the actuators: torque_on 
- set the torque limits to a very small number (e.g. 0.001  - 0.1%)

For example, you can use the following commands:
```bash
ros2 topic pub --once /forward_torque_controller/commands std_msgs/Float64MultiArray "{ data:[1,1,1,1,1,1,1,1,1,1,1]}"
ros2 topic pub --once /forward_torque_limit_controller/commands std_msgs/Float64MultiArray "{data:[0.001, 0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001,0.001]}"
```


Or you cane use the script provided in the `scripts` folder:

```bash
sh make_compliant.sh
```

The node will compensate for the gravity of both arms and allows setting an additional weight at the end-effector of each arm using the topic `/set_payload_mass`.
This topic receives two float64 values representing the mass (in kg) to be added to the left and right arms, respectively. 



For example to set a payload of 0.5 kg on the left arm and 1.0 kg on the right arm, you can use the following command:

```bash
ros2 topic pub /set_payload_mass std_msgs/msg/Float64MultiArray "{data: [0.5, 1.0]}"
```

or you can also use a script provided in the `scripts` folder:

```bash
sh set_payload.sh 0.5 1.0
```

The node also allows to estimate the current payload mass of each arm which is published continuously on the topic `/current_payload_mass` as a Float64MultiArray message containing two values: the estimated mass (in kg) for the left and right arms, respectively.

To run the node manually, you can use the following command:

```bash
ros2 topic echo /current_payload_mass
```


