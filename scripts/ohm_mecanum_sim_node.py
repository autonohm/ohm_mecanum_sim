#!/usr/bin/env python3

# ------------------------------------------------------------
# Author:      Stefan May
# Date:        20.4.2020
# Description: Pygame-based robot simulator application
# ------------------------------------------------------------

import pygame
import rospy
from ohm_mecanum_simulator import Ohm_Mecanum_Simulator
from robot import Robot
from fleet import Fleet

import numpy as np
from math import cos, sin, pi, sqrt, acos

pygame.init()

# Screen size
size = width, height = 1600, 900

# Drawing surface
surface = pygame.display.set_mode(size, pygame.HWSURFACE | pygame.DOUBLEBUF)

sim = Ohm_Mecanum_Simulator(surface, "ohm_mecanum_sim", "Ohm Mecanum Simulator")

# Robot 1
x1     = 1
y1     = 0
theta1 = 0
T1 = np.matrix([[cos(theta1), -sin(theta1), x1],
                [sin(theta1), cos(theta1), y1],
                [0, 0, 1]])

x2     = -1
y2     = 0
theta2 = pi/2
T2 = np.matrix([[cos(theta2), -sin(theta2), x2],
                [sin(theta2), cos(theta2), y2],
                [0, 0, 1]])

x3     = 0
y3     = 2
theta3 = pi/4
T3 = np.matrix([[cos(theta3), -sin(theta3), x3],
                [sin(theta3), cos(theta3), y3],
                [0, 0, 1]])

x4     = 0
y4     = -2
theta4 = -pi/4
T4 = np.matrix([[cos(theta4), -sin(theta4), x4],
                [sin(theta4), cos(theta4), y4],
                [0, 0, 1]])

# Common kinematic center
x_fleet = 8
y_fleet = 5
theta_fleet = 0

T_fleet = np.matrix([[cos(theta_fleet), -sin(theta_fleet), x_fleet],
                        [sin(theta_fleet), cos(theta_fleet), y_fleet],
                        [0, 0, 1]])

fleet = Fleet(T_fleet, "fleet1")
fleet.add_robot(T1, "robot1")
fleet.add_robot(T2, "robot2")
fleet.add_robot(T3, "robot3")
fleet.add_robot(T4, "robot4")
sim.add_fleet(fleet)

sim.run()
rospy.spin()
