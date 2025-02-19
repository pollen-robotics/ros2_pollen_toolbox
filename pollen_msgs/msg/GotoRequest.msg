# Total duration of the movement
float64 duration

# Goal state of the joints. In normal use, this is the last command that will be sent to the robot. Non 0 velocities can be used to link gotos requests without stopping at each goal.
sensor_msgs/JointState goal_joints

# Goal pose for cartesian interpolation request
geometry_msgs/PoseStamped goal_pose

# 2 spaces are supported: "cartesian" and joints"
string interpolation_space

# As of writting 3 modes are supported: "linear", "minjerk" and "elliptical".
string mode

# The sampling frequency of the trajectory. If not set, the action server will chose the sampling frequency.
float64 sampling_freq

# For elliptical interpolation only.
string arc_direction
float64 secondary_radius

# *experimental* If set to true, the movement will be stopped if an anomaly is detected along the trajectory.
bool safety_on
