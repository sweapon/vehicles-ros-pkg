#!/bin/bash
# Ublox configuration
export UBLOX_PORT=/dev/ttyACM0
export UBLOX_BAUD=115200
export UBLOX_USE_GPS=true

# Spatial configuration 
export SPATIAL_PORT=/dev/ttyUSB0
export SPATIAL_BAUD=115200
export SPATIAL_USE_GPS=false

# NavQuest DVL configuration
export NAVQUEST_PORT=/dev/ttyUSB3
export NAVQUEST_BAUD=115200
export NAVQUEST_FIXED_YAW=3.14159
export NAVQUEST_USE_FIXED_YAW=true

# Seatrac USBL configuration
export USBL_PORT=/dev/ttyUSB2
export USBL_BAUD=115200

# Novatel dGPS configuration
export NOVATEL_PORT=/dev/ttyS1
export NOVATEL_BAUD=115200
export USE_NOVATEL_GPS=false

# Arduino driver configuration
export ARDUINO_PORT=/dev/ttyACM1
export ARDUINO_BAUD=115200

# VideoRay driver configuration
export VR_PORT=/dev/ttyS2
export VR_BAUD=115200

# Relay Module configuration
export RELAY_MODULE_IP=pladypos-eth
export RELAY_MODULE_PORT=17494
export RELAY_MODULE2_IP=pladypos-eth2
export RELAY_MODULE2_PORT=17494
