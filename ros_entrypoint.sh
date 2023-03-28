#!/bin/bash
set -e

# source ros & catkin environment 
source "/opt/ros/$ROS_DISTRO/setup.bash"

echo "Catkin sourced"

# source workspace it it exist
SOURCE_WS_FILE=/ros_ws/devel/setup.bash
if [ -f $SOURCE_WS_FILE ]; then
    source $SOURCE_WS_FILE
    echo "Worskspace sourced"
fi

DEFAULT_TURTLEBOT=waffle_pi

export TURTLEBOT3_MODEL=$DEFAULT_TURTLEBOT

echo "Default turtlebot model is set to $DEFAULT_TURTLEBOT."

exec "$@"