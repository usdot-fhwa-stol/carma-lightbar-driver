#!/bin/bash

# Read username and password from YAML config file
CONFIG_FILE="/opt/carma/vehicle/calibration/lightbar/auth_config.yaml"
USER=$(grep "user:" "$CONFIG_FILE" | cut -d "'" -f 2)
PASS=$(grep "password:" "$CONFIG_FILE" | cut -d "'" -f 2)

# HTTP request to turn off all lightbar patterns. Different DOs indicate different pre-configured lightbar patterns, 1 means OFF 0 means ON.
curl -X POST -d "DO0=1&DO1=1&DO2=1&DO3=1&DO4=1&DO5=1&DO6=1&DO7=1&DO8=1&DO9=1&DO10=1&DO11=1&DO12=1&DO13=1&DO14=1&DO15=1" -u "$USER:$PASS" http://192.168.88.28:80/digitaloutput/all/value
echo "All lights turned OFF"
