cmake_minimum_required(VERSION 3.8)
project(pollen_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Gripper.msg"
  "msg/ReachableGraspTriplet.msg"
  "msg/GraspPose.msg"
  "msg/ReachableGraspTripletArray.msg"
  "msg/GraspPoseArray.msg"
  "msg/GotoFeedback.msg"
  "msg/GotoRequest.msg"
  "msg/GotoResult.msg"
  "msg/IKRequest.msg"
  "msg/ReachabilityState.msg"

  "action/Goto.action"

  "srv/GetDynamicState.srv"
  "srv/GetInverseKinematics.srv"
  "srv/GetReachableGraspTriplets.srv"
  "srv/GetForwardKinematics.srv"
  "srv/GetObjectDetection.srv"
  "srv/GetSimpleGraspPose.srv"
  "srv/GetGraspCNN.srv"
  "srv/GetReachability.srv"
  

  DEPENDENCIES geometry_msgs sensor_msgs std_msgs trajectory_msgs vision_msgs
)

ament_package()




