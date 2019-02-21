#!/bin/bash
# bash script for starting robot controller code

#SCRIPTPATH=/home/create/apriltag/python/robot_controller.py
#SCRIPTPATH=/home/create/apriltag/python/robot_controller_udp_MP.py
SCRIPTPATH=/home/create/apriltag/python/RSonCreate.py

echo "Starting robot script..."
sudo nice -n -5 python3 $SCRIPTPATH
