# ------------------------------------------------------------------------
# Author:      Stefan May
# Date:        20.4.2020
# Description: Pygame-based robot representation for the mecanum simulator
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
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ohm_mecanum_sim.msg import WheelSpeed

class Robot:

    # Linear velocity in m/s
    _v            = [0, 0]

    # Angular velocity in rad/s
    _omega              = 0

    # Radius of wheels
    _wheel_radius       = 0.05

    # Maximum angular rate of wheels in rad/s
    _wheel_omega_max    = 10

    # Center distance between front and rear wheels
    _wheel_base         = 0.3

    # Distance between left and right wheels
    _track              = 0.2

    # Zoomfactor of image representation
    _zoomfactor         = 1.0

    # Animation counter, this variable is used to switch image representation to pretend a driving robot
    _animation_cnt      = 0

    def __init__(self, T, name):
        self._T_pose_init = T
        self._T_pose = self._T_pose_init

        self._reset = False
        self._coords = [self._T_pose[0, 2], self._T_pose[1, 2]]
        self._theta = atan2(self._T_pose[1, 0], self._T_pose[0, 0])
        self._lock = threading.Lock()

        # Matrix of kinematic concept
        lxly = (self._wheel_base/2 + self._track/2) / self._wheel_radius
        rinv = 1/self._wheel_radius
        self._T_kinematic = np.matrix([[rinv, -rinv, -lxly],
                            [-rinv, -rinv, -lxly],
                            [ rinv,  rinv, -lxly],
                            [-rinv,  rinv, -lxly]])
                            
        # Inverse of matrix is used for setting individual wheel speeds
        self._Tinv_kinematic = np.linalg.pinv(self._T_kinematic)

        # Calculate maximum linear speed in m/s
        self._max_speed = self._wheel_omega_max * self._wheel_radius

        # Calculate maximum angular rate of robot in rad/s
        self._max_omega = self._max_speed / (self._wheel_base/2 + self._track/2)

        self._name              = name
        img_path                = os.path.join(os.path.dirname(__file__), "../images/mecanum_edu_1.png")
        img_path2               = os.path.join(os.path.dirname(__file__), "../images/mecanum_edu_2.png")
        img_path_crash          = os.path.join(os.path.dirname(__file__), "../images/mecanum_crash_2.png")
        self._symbol            = pygame.image.load(img_path)
        self._symbol2           = pygame.image.load(img_path2)
        self._symbol_crash      = pygame.image.load(img_path_crash)
        self._img               = pygame.transform.rotozoom(self._symbol, self._theta, self._zoomfactor)
        self._img2              = pygame.transform.rotozoom(self._symbol2, self._theta, self._zoomfactor)
        self._img_crash         = pygame.transform.rotozoom(self._symbol_crash, self._theta, self._zoomfactor)
        self._robotrect         = self._img.get_rect()
        self._robotrect.center  = self._coords
        self._pub_pose          = rospy.Publisher(str(self._name)+"/pose", PoseStamped, queue_size=1)
        self._pub_odom          = rospy.Publisher(str(self._name)+"/odom", Odometry, queue_size=1)

        self._run               = True
        self._thread            = threading.Timer(0.1, self.trigger)
        self._thread.start()
        self._timestamp         = rospy.Time.now()
        self._last_command      = self._timestamp

    def __del__(self):
        self.stop()

    def reset_pose(self):
        self._reset = True

    def set_max_velocity(self, vel):
        self._max_speed = vel

    def set_wheel_speed(self, omega_wheel):
        w = np.array([omega_wheel[0], omega_wheel[1], omega_wheel[2], omega_wheel[3]])
        res = self._Tinv_kinematic.dot(w)
        self.set_velocity(res[0,0], res[0,1], res[0,2])

    def set_velocity(self, vx, vy, omega):
        x = np.array([vx, vy, omega])
        omega_i = self._T_kinematic.dot(x)
        self._v = [vx, vy]
        self._omega = omega

    def acquire_lock(self):
        self._lock.acquire()

    def release_lock(self):
        self._lock.release()

    def stop(self):
        self.set_velocity(0, 0, 0)
        self._run = False

    def trigger(self):
        while(self._run):
            self.acquire_lock()

            # Measure elapsed time
            timestamp = rospy.Time.now()#time.process_time()
            elapsed = (timestamp - self._timestamp).to_sec()
            self._timestamp = timestamp

            # SM: Uncommented for flexible kinematic evaluation
            # Check, whether commands arrived recently
            last_command_arrival = timestamp - self._last_command
            #if last_command_arrival.to_sec() > 0.5:
            #    self._v[0] = 0
            #    self._v[1] = 0
            #    self._omega = 0

            # Change orientation
            self._theta += self._omega * elapsed

            # Transform velocity vectors to global frame
            cos_theta = cos(self._theta)
            sin_theta = sin(self._theta)
            v =   [self._v[0], self._v[1]]
            v[0] = cos_theta*self._v[0] - sin_theta * self._v[1]
            v[1] = sin_theta*self._v[0] + cos_theta * self._v[1]

            # Move robot
            self._coords[0] += v[0]  * elapsed
            self._coords[1] += v[1]  * elapsed

            # Publish pose
            p = PoseStamped()
            p.header.frame_id = "pose"
            p.header.stamp = self._timestamp
            p.pose.position.x = self._coords[0]
            p.pose.position.y = self._coords[1]
            p.pose.position.z = 0
            p.pose.orientation.w = cos(self._theta/2.0)
            p.pose.orientation.x = 0
            p.pose.orientation.y = 0
            p.pose.orientation.z = sin(self._theta/2.0)
            self._pub_pose.publish(p)

            # Publish odometry
            o = Odometry()
            o.header.frame_id ="odom"
            o.header.stamp = self._timestamp
            o.pose.pose.position = p.pose.position
            o.pose.pose.orientation = p.pose.orientation
            o.child_frame_id = "base_link"
            o.twist.twist.linear.x = v[0];
            o.twist.twist.linear.y = v[1];
            o.twist.twist.angular.z = self._omega;
            self._pub_odom.publish(o)

            if(self._reset):
                time.sleep(1.0)
                self._T_pose = self._T_pose_init
                self._coords = [self._T_pose[0, 2], self._T_pose[1, 2]]
                self._theta = atan2(self._T_pose[1, 0], self._T_pose[0, 0])
                self._reset = False

            self.release_lock()
            time.sleep(0.04)

    def get_coords(self):
        return self._coords

    def get_rect(self):
        self._img       = pygame.transform.rotozoom(self._symbol,       (self._theta-pi/2)*180.0/pi, self._zoomfactor)
        self._img2      = pygame.transform.rotozoom(self._symbol2,      (self._theta-pi/2)*180.0/pi, self._zoomfactor)
        self._img_crash = pygame.transform.rotozoom(self._symbol_crash, (self._theta-pi/2)*180.0/pi, self._zoomfactor)
        self._robotrect = self._img.get_rect()
        return self._robotrect

    def get_image(self):
        if(not self._reset):
            self._animation_cnt += 1
        magnitude = abs(self._v[0])
        if(abs(self._v[1]) > magnitude):
            magnitude = abs(self._v[1])
        if(abs(self._omega)>magnitude):
            magnitude = abs(self._omega)
        if magnitude < 0.5:
            moduloval = 6
        else:
            moduloval = 2
        
        if(self._reset):
            return self._img_crash
        elif(self._animation_cnt % moduloval < moduloval/2 and (self._v[0]!=0 or self._v[1]!=0 or self._omega!=0)):
            return self._img
        else:
            return self._img2


