#!/bin/bash
# bash script for starting robot controller code in lab mode

SCRIPTPATH=/home/create/apriltag/python/RSonCreate.py

echo "Starting robot script..."
sudo nice -n -5 python3 $SCRIPTPATH
