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
            
            pose = robot._T_pose

            # calculate pose relative to kinematic center
            T_pose_inv = np.linalg.pinv(self._T_pose)
            pose_rel = T_pose_inv * pose
            
            # isolate orientation from robot pose
            rp = pose
            #rp[0, 2] = 0
            #rp[1, 2] = 0
            
            # invert it
            rpi = np.linalg.pinv(rp)

            # calculate translational part in coordinate system of fleet
            vtx = rpi[0, 0] * vx + rpi[0, 1] * vy
            vty = rpi[1, 0] * vx + rpi[1, 1] * vy

            nx = pose[0, 2] - self._T_pose[0, 2]
            ny = pose[1, 2] - self._T_pose[1, 2]
            
            vnx = -ny * omega
            vny = nx * omega
            vnx2 = rpi[0, 0] * vnx + rpi[0, 1] * vny
            vny2 = rpi[1, 0] * vnx + rpi[1, 1] * vny
    
            robot.set_velocity(vtx+vnx2, vty+vny2, omega)

        self._last_command = rospy.Time.now()