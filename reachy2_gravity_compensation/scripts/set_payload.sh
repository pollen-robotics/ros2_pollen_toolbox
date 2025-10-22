# receives payload mass (kg) for two arms and publish it to the appropriate topic
# Usage: ./set_payload.sh  <left_arm_payload_kg> <right_arm_payload_kg>
# if no argument is given, both payloads are set to 0 kg

if [ -z "$1" ]; then
    LEFT_PAYLOAD=0.0
    else
    LEFT_PAYLOAD=$1
    fi
if [ -z "$2" ]; then
    RIGHT_PAYLOAD=0.0
    else
    RIGHT_PAYLOAD=$2
    fi

ros2 topic pub --once /set_payload_mass std_msgs/Float64MultiArray "{ data:[$LEFT_PAYLOAD, $RIGHT_PAYLOAD]}"