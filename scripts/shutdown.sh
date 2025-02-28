#!/bin/bash
curl -X POST -d "DO0=1&DO1=1&DO2=1&DO3=1&DO4=1&DO5=1&DO6=1&DO7=1&DO8=1&DO9=1&DO10=1&DO11=1&DO12=1&DO13=1&DO14=1&DO15=1" -u root:00000000 http://192.168.88.28:80/digitaloutput/all/value
echo "All lights turned OFF"
