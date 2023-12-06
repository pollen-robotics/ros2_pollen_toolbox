std_msgs/Header header

# status of the current action. Currently: "running" or "safety"
string status

# remaining time until completion in seconds
float64 time_to_completion
# number of commands sent since last status message
int32 commands_sent
