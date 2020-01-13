# Raspberry Pi

Bash Script 'PyUpdate' sits in the home folder.

To download PyUpdate directly from git:

- SSH into the Pi with Putty. In the home directory run:

- rm PyUpdate

- wget https://raw.githubusercontent.com/autonomousmobilerobots/RaspberryPi/master/PyUpdate

- chmod +x PyUpdate

.

Run ./PyUpdate to download the current 'RSonCreate.py', 'robot_controller.py', 'robot' and 'debug' scripts from Git.

. Bash script 'robot' sits in the home folder and calls the script that controls the robot and camera in regular lab mode.

. Bash script 'debug' sits in the home folder and calls the script that controls the robot and camera in debug mode.

. robot_controller.py and RSonCreate.py are Python scripts that sit in /apriltag/python/     

. robot_controller.py is used for debug. it gives access to grayscale and depth images as well as commanding the create.

. RSonCreate.py is the script used in lab.

. robot_controller_udp.py and robot_controller_udp_MP.py are older versions not currently used.
