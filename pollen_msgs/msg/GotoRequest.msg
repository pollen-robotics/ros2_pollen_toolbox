# Total duration of the movement
float64 duration

# Starting state of the joints. If empty, the current state is read and used. Non 0 velocities can be used to link gotos requests without stopping at each goal.
sensor_msgs/JointState starting_joints

# Goal state of the joints. In normal use, this is the last command that will be sent to the robot. Non 0 velocities can be used to link gotos requests without stopping at each goal.
sensor_msgs/JointState goal_joints
# As of writting 2 modes are supported: "linear" and "minjerk".
string mode

# The sampling frequency of the trajectory. If not set, the action server will chose the sampling frequency.
float64 sampling_freq

# *experimental* If set to true, the movement will be stopped if an anomaly is detected along the trajectory.
bool safety_on
