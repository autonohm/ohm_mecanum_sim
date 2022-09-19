# ------------------------------------------------------------------------
# Author:      Stefan May
# Date:        18.9.2022
# Description: Pygame-based robot fleet representation for the mecanum simulator
# ------------------------------------------------------------------------

import os
import pygame
import rospy
import time, threading
import operator
import numpy as np
from math import cos, sin, pi, sqrt, acos, atan2
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from ohm_mecanum_sim.msg import WheelSpeed
from ohm_mecanum_sim.srv import Spawn, Kill, SpawnRequest, SpawnResponse, KillRequest, KillResponse
from robot import Robot

class Fleet:

    def __init__(self, T, name):
        self._T_pose_init = T
        self._T_pose  = self._T_pose_init
        self._name = name

        self._robots = []

        rospy.Service('/spawn', Spawn, self.service_callback_spawn)
        rospy.Service('/kill', Kill, self.service_callback_kill)

        self._lock    = threading.Lock()
        self._sub_joy = rospy.Subscriber(str(self._name)+"/joy", Joy, self.callback_joy)

    def get_coords(self):
        return [self._T_pose[0, 2], self._T_pose[1, 2]]

    def add_robot(self, T, name):
        T_robot = self._T_pose * T
        self._robots.append(Robot(T_robot, name))

    def get_robots(self):
        return self._robots

    def service_callback_spawn(self, req):
        self.spawn_robot(req.x, req.y, req.theta, req.name)
        response = SpawnResponse(req.x, req.y, req.theta, req.name)
        return response

    def service_callback_kill(self, req):
        self.kill_robot(req.name)
        response = KillResponse(True)
        return response

    def callback_joy(self, data):
        #TODO: check for max speed of all robots
        vx = data.axes[1]
        vy = data.axes[0]
        omega = data.axes[2]
        for robot in self._robots:
            
            T_pose_fleet          = self._T_pose
            T_pose_robot_to_fleet = robot._T_pose_init
            T_pose_r2f_inv = np.linalg.pinv(T_pose_robot_to_fleet)
            print(robot._T_pose)

            # calculate translational part in coordinate system of robot
            vtx_robot = T_pose_r2f_inv[0, 0] * vx + T_pose_r2f_inv[0, 1] * vy
            vty_robot = T_pose_r2f_inv[1, 0] * vx + T_pose_r2f_inv[1, 1] * vy

            # calculate translation vector between kinematic center of fleet and robot position
            nx = T_pose_robot_to_fleet[0, 2] - T_pose_fleet[0, 2]
            ny = T_pose_robot_to_fleet[1, 2] - T_pose_fleet[1, 2]
            
            # the perpendicular vector (normal) points to the direction, where a rotation around the fleet's kinematic center wourld lead us.
            vnx = -ny * omega
            vny = nx * omega

            # now, translate this movement in the global coordinate system
            vwx_robot = T_pose_r2f_inv[0, 0] * vnx + T_pose_r2f_inv[0, 1] * vny
            vwy_robot = T_pose_r2f_inv[1, 0] * vnx + T_pose_r2f_inv[1, 1] * vny
    
            # and add it to the translational part
            robot.set_velocity(vtx_robot+vwx_robot, vty_robot+vwy_robot, omega)

        self._last_command = rospy.Time.now()