#!/bin/bash
# ROS Environment
USER=stdops
VEHICLE=buddy
ROS_HOME=/home/${USER}/ros
source ${ROS_HOME}/devel/setup.bash
export ROS_MASTER_URI=http://buddy:11311

# Vehicle configuration environment
source `rospack find ${VEHICLE}`/data/devices/device_config.bash
source `rospack find ${VEHICLE}`/data/control/control_config.bash
source `rospack find ${VEHICLE}`/data/navigation/navigation_config.bash

# Launch configuration
export LAUNCH_PKG=buddy
export LAUNCH_FILE=buddy_standard_mws.launch

# Configure logging
export LOG_PATH=/mnt/extern/logs/launcher
export ROS_LOG_DIR=/mnt/extern/logs/ros
export LOGGING=true
