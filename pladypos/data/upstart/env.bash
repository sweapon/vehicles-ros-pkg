#!/bin/bash
# ROS Environment
USER=stdops
VEHICLE=pladypos
ROS_HOME=/home/${USER}/ros
source ${ROS_HOME}/devel/setup.bash

# Vehicle configuration environment
source `rospack find ${VEHICLE}`/data/devices/device_config.bash
source `rospack find ${VEHICLE}`/data/control/control_config.bash
source `rospack find ${VEHICLE}`/data/navigation/navigation_config.bash

# Launch configuration
LAUNCH_PKG=pladypos
LAUNCH_FILE=pladypos_standard_mws.launch

# Configure logging
LOG_PATH=/extern/launcher
ROS_LOG_PATH=/extern/ros
