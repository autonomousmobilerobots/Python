# Raspberry Pi

Bash script 'robot' sits in the home folder and calls the script that controls the robot and camera in regular lab mode.

Bash script 'debug' sits in the home folder and calls the script that controls the robot and camera in debug mode.

Script 'PyUpdate' sits in the home folder. Run ./PyUpdtae to get the current 'RSonCreate.py', 'robot_controller.py', 'robot' and 'debug' scripts from Git.

Trobot_controller.py and RSonCreate.py are Python scripts that sit in /apriltag/python/     

robot_controller.py is used for debug. it gives access to grayscale and depth images as well as commanding the create.

RSonCreate.py is the script used in lab.
