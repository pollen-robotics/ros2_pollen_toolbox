# Simple Goto action server

- exposes 2 action topics:
  - /right_arm_goto
  - /left_arm_goto

## Examples

`ros2 run reachy_trajectory goto`

`ros2 action send_goal /right_arm_goto pollen_msgs/action/GotoTrajectory "{ trajectory: { joint_names: [r_shoulder_pitch], points: [ {positions: [-1], time_from_start: {sec: 2}} ] } }" --feedback`


ros2 action send_goal /right_arm_goto pollen_msgs/action/GotoTrajectory "{ trajectory: { joint_names: [r_elbow_pitch], points: [ {positions: [-1], time_from_start: {sec: 2}} ] } }" --feedback

## 27/11/2023 debug Rémi
Fixed the bugs that bloqued the async calls on the Python client side.
Now this works:
```
ros2 launch reachy_bringup reachy.launch.py  fake:=true start_rviz:=true start_sdk_server:=true
```

```
ros2 run reachy_trajectory goto
```

```
cd ~/reachy_ws/src/pollen_grasping/reachy_trajectory/scripts
python3 test_client_new.py
```

TODOs:
- Sometimes when calling the client code often, a goal is not executed. => I think what happens is that sometimes the goals are inverted (so the 0 goal is executed first and the -1 goal after). If true, this is OK behaviour probably.
- Test all features :
  - cancellability
  - several joints
  - continuity in position and speed
- Assess if we want to change the messages used
- Cleanefy the code and move it in the correct places

