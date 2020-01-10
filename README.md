# Raspberry Pi

Bash script 'robot' sits in the home folder and calls the script that controls the robot and camera in regular lab mode.
Bash script 'debug' sits in the home folder and calls the script that controls the robot and camera in debug mode.

Script 'PyUpdate' sits in the home folder. Run ./PyUpdtae to get the current RSonCreate.py and robot_controller.py scripts from Git.

The other scripts are written in Python and sit in /apriltag/python/     

Robot_controller.py is used for debug. it gives access to grayscale and depth images as well as commanding the create.

RSonCreate.py is the script used in lab.
