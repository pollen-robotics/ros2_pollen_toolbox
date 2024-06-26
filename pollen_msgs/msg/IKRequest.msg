# 6d goal pose
geometry_msgs/PoseStamped pose

# Current options are "unconstrained" and "low_elbow"
string constrained_mode

# Value in [-pi, pi], angle of the elbow that the IK will try to obtain for the right arm
# Common value : -4pi/6
float64 preferred_theta

# When in continuous mode this is the maximum amount of rads that the elbow can rotate on the reachability circle
# Common value : 0.01
float64 d_theta_max

# Current options are "continuous" and "discrete"
string continuous_mode

# When present this value must be unic for each message and it will be used to identify reachability information for a given request
int64 order_id
