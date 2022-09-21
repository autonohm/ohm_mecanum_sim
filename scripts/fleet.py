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
from copy import copy, deepcopy

class Fleet:

    def __init__(self, T, name):
        self._T_pose  = deepcopy(T)
        self._T_r2f_ref = np.matrix([[1, 0, 0],
                                     [0, 1, 0],
                                     [0, 0, 1]])
        
        self._name = name

        self._robots = []

        rospy.Service(str(self._name)+"/spawn", Spawn, self.service_callback_spawn)
        rospy.Service(str(self._name)+"/kill", Kill, self.service_callback_kill)

        self._lock    = threading.Lock()
        self._sub_joy = rospy.Subscriber(str(self._name)+"/joy", Joy, self.callback_joy)

    def get_coords(self):
        return [self._T_pose[0, 2], self._T_pose[1, 2]]

    def add_robot(self, T, name):
        T_robot = self._T_pose * T
        if (len(self._robots)==0):
            self._T_r2f_ref = deepcopy(T)
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
        
        if (len(self._robots)==0):
            return

        vx = data.axes[1]
        vy = data.axes[0]
        omega = data.axes[2]

        
        T_r2f_ref_inv = np.linalg.pinv(self._T_r2f_ref)
        T_pose_fleet = self._robots[0]._T_pose * T_r2f_ref_inv
        T_pose_fleet_inv = np.linalg.pinv(T_pose_fleet)

        for robot in self._robots:
            
            T_pose_r2f = T_pose_fleet_inv * robot._T_pose
            T_pose_r2f_inv = np.linalg.pinv(T_pose_r2f)
            
            #print(T_pose_r2f)
            #print(T_pose_fleet)

            # calculate translational part in coordinate system of robot
            nx = -T_pose_r2f[1, 2] * omega
            ny = T_pose_r2f[0, 2] * omega
            vtx_robot = T_pose_r2f_inv[0, 0] * (vx+nx) + T_pose_r2f_inv[0, 1] * (vy+ny)
            vty_robot = T_pose_r2f_inv[1, 0] * (vx+nx) + T_pose_r2f_inv[1, 1] * (vy+ny)
    
            # and add it to the translational part
            robot.set_velocity(vtx_robot, vty_robot, omega)

        self._last_command = rospy.Time.now()