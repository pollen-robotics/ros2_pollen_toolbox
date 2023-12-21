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
- Interpolation methods: linear or minimum jerk.
- Interpolation frequency: the server will apply the frequency requested by the client.

## Examples

Python examples on different ways to make client calls [here](./pollen_goto/goto_action_client.py)
To run the server:
```
ros2 run pollen_goto goto_server
```
To run the client examples:
```
ros2 run pollen_goto goto_client_test
```
