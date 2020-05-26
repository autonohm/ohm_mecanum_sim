# ohm_mecanum_sim
This package comprises a pygame-based robot simulation for mecanum-driven kinematic concepts.

![](images/screenshot 1-1.png?raw=true)

Tests have been performed with ROS melodic, albeit the used python versions differ. While ROS melodic uses python2.x, the simulator need python3. The  reason is, that ROS noetic will require python3. In order to make the simulator work with ROS melodic, you can follow the installation hints below.

## Prerequisites on Ubuntu 18.4
Installing python3 aside python2 can be done with pip:
```console
user@machine:~$ sudo apt install python3 python-pip python3-pip python-catkin-tools
user@machine:~$ sudo update-alternatives --install /usr/bin/python python /usr/bin/python2.7 1
user@machine:~$ sudo update-alternatives --install /usr/bin/python python /usr/bin/python3.6 2
user@machine:~$ pip install -U defusedxml rospkg pygame pycryptodomex gnupg
```
## Starting the simulator
Go to your catkin workspace / execute the following commands (replace the path to your catkin workspace accordingly):
```console
user@machine:~$ roscore &
user@machine:~$ cd workspace/catkin_ws
user@machine:~/workspace/catkin_ws$ catkin_make
user@machine:~/workspace/catkin_ws$ source devel/setup.bash
user@machine:~/workspace/catkin_ws$ python3 src/ohm_mecanum_sim/scripts/ohm_mecanum_sim_node.py
```

## Moving around the robots
One can use different tools to make the robot move:
- Publish joy messages to a certain robot instance (if you have a joystick)
```console
user@machine:~$ rosrun joy joy_node joy:=/robot1/joy
```
- Publish a twist message manually by using rostopic pub
```console
user@machine:~$ rostopic pub -r 10 /robot1/cmd_vel geometry_msgs/Twist ...
```
- Use a keyboard tool, like teleop_twist_keyboard (install ros-melodic-teleop-twist-keyboard first)
```console
user@machine:~$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/robot1/cmd_vel
```
- ... or write your own node.
