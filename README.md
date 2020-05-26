# ohm_mecanum_sim
This package comprises a pygame-based robot simulation for mecanum-driven kinematic concepts. Tests have been performed with ROS melodic, albeit the used python versions differ. While ROS melodic uses python2.x, the simulator need python3. The  reason is, that ROS noetic will require python3. In order to make the simulator work with ROS melodic, you can follow the installation hints below.

## Prerequisites on Ubuntu 18.4
Installing python3 aside python2 can be done with pip:
$ sudo apt install python3 python-pip python3-pip python-catkin-tools python3-catkin-tools
$ sudo update-alternatives --install /usr/bin/python python /usr/bin/python2.7 1
$ sudo update-alternatives --install /usr/bin/python python /usr/bin/python3.6 2
$ pip install -U defusedxml rospkg pygame pycryptodomex gnupg

## Starting the simulator
Go to your catkin workspace and execute ...
$ roscore &
$ source devel/setup.bash
$ python3 src/ohm_mecanum_simulator/scripts/ohm_mecanum_simulator_node.py


