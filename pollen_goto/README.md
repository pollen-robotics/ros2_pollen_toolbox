# Pollen Goto

At its basic level, the goto action server will perform interpolations in joint state, the resulting joint commands are published on topic ```/dynamic_joint_commands```.

Exposes 3 action servers:
  - /r_arm_goto_action_server
  - /l_arm_goto_action_server
  - /neck_goto_action_server

## Features
- **Async.** Gotos can be call in a blocking manner or in async
- **Cancelation.** Goals can be canceled at any moment during execution
- **Queuing.** If a goal is received during the execution of a goto, it will be queued and executed as soon as the previous gotos are finished. 
  => This can be used to obtain continuous movements between gotos by dynamically adding a goto request whose start state matches the end state of the previous goto 

Options:
- Interpolation methods: linear, minimum jerk, or sinusoidal
- Interpolation frequency: the server will apply the frequency requested by the client.

## Examples
Python examples on different ways to make client calls [here](./pollen_goto/goto_action_client.py)

### Using Sinusoidal Interpolation
```python
from pollen_msgs.action import Goto
from pollen_goto.interpolation import JointSpaceInterpolationMode

# Create a goto request
goto_request = Goto.Request()
goto_request.interpolation_mode = JointSpaceInterpolationMode.SINUSOIDAL_FUNC
goto_request.duration = 2.0  # 2 seconds duration
goto_request.joints = ["r_shoulder_pitch", "r_shoulder_roll"]
goto_request.positions = [0.5, 0.3]  # Target positions

# Send the request to the action server
# This will create a smooth, natural-looking motion
```

To run the server:
```
ros2 run pollen_goto goto_server
```
To run the client examples:
```
ros2 run pollen_goto goto_client_test
```

## References
Learn more about ROS2 Actions:
https://design.ros2.org/articles/actions.html
https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html

