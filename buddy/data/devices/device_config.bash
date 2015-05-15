#!/bin/bash
# Ublox configuration
export UBLOX_PORT=/dev/ttyUSB1
export UBLOX_BAUD=115200
export UBLOX_USE_GPS=true

# NavQuest DVL configuration
export NAVQUEST_PORT=/dev/ttyS0
export NAVQUEST_BAUD=115200
export NAVQUEST_FIXED_YAW=3.14159
export NAVQUEST_USE_FIXED_YAW=true

# Seatrac USBL configuration
export USBL_PORT=/dev/ttyUSB0
export USBL_BAUD=115200

# Microstrain IMU
export MICROSTRAIN_PORT=/dev/ttyACM1
export MICROSTRAIN_BAUD=115200

# Arduino diagnostics
export ARDUINO_PORT=/dev/ttyACM0
export ARDUINO_BAUD=115200

# Relay Module configuration
#export RELAY_MODULE_IP=pladypos-eth
#export RELAY_MODULE_PORT=17494
#export RELAY_MODULE2_IP=pladypos-eth2
#export RELAY_MODULE2_PORT=17494
