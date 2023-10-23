# Pollen Placo Kinematics node

### Kinematics computation service

It exposes services for kinematics (forward and inverse) computations:

* **/r_arm/forward_kinematics_placo** ([GetForwardKinematics.srv](../pollen_msgs/srv/GetForwardKinematics.srv)) - Compute the forward kinematics for the right arm. 7 joints should be provided (r_shoulder_pitch, r_shoulder_roll, r_elbow_yaw, r_elbow_pitch, r_wrist_roll, r_wrist_pitch, r_wrist_roll).
* **/r_arm/inverse_kinematics_placo** ([GetInverseKinematics.srv](../pollen_msgs/srv/GetInverseKinematics.srv)) - Compute the inverse kinematics for the right arm.
* **/r_arm/reachability_placo** ([GetReachability.srv](../pollen_msgs/srv/GetReachability.srv)) - Check the reachability of a pose and return the inverse kinematics for the right arm.

* **/l_arm/forward_kinematics_placo** ([GetForwardKinematics.srv](../pollen_msgs/srv/GetForwardKinematics.srv)) - Compute the forward kinematics for the left arm. 7 joints should be provided (l_shoulder_pitch, l_shoulder_roll, l_elbow_yaw, l_elbow_pitch, l_wrist_roll, l_wrist_pitch, l_wrist_roll).
* **/l_arm/inverse_kinematics_placo** ([GetInverseKinematics.srv](../pollen_msgs/srv/GetInverseKinematics.srv)) - Compute the inverse kinematics for the left arm.
* **/l_arm/reachability_placo** ([GetReachability.srv](../pollen_msgs/srv/GetReachability.srv)) - Check the reachability of a pose and return the inverse kinematics for the left arm.


### Cartesian control

The node can also be used as an cartesian controller. Indeed, it listens to specific cartesian targets, compute the corresponding joints commands using the inverse kinematics. Then, it publishes those joints commands directly in the corresponding forward_position controller.

For each arm, two topics can be used (for the right arm):

* **/r_arm/target_pose_placo** ([PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html)) - Compute the inverse kinematics of the given pose and directly send the joint solution to the corresponding forward position controller.
* **/r_arm/averaged_target_pose_placo** ([PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html)) - Average the pose (over the n last), compute the inverse kinematics, clip the joint solution according to a max velocity and then publish the result to the corresponding forward posiiotn controller. This version is typically meant to be used at a rather high frequency (> 10Hz).

Similarly for the left arm: **/l_arm/target_pose_placo** and **/l_arm/averaged_target_pose_placo**.

## Requirements

Please note that in order to work properly, this node requires the "/robot_description" and "/joint_states" topics to be published. Depending on your URDF, only the corresponding kinematics chain and their associated services/topics will be created.

## Install

sudo apt install python3-pykdl

Install locally reachy_placo:
```
cd ~/dev
git clone git@github.com:pollen-robotics/reachy_placo.git
cd reachy_placo
pip install -e .
```
-> The intallation of placo and pinnochio can be done manually, but the current way of doing it is using a pre installed Docker image.