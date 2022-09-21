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

######### Fleet 1 ########
# Robot 1
x1     = 1
y1     = 0
theta1 = pi/2
T1 = np.matrix([[cos(theta1), -sin(theta1), x1],
                [sin(theta1), cos(theta1), y1],
                [0, 0, 1]])

# Robot 2
x2     = -1
y2     = 0
theta2 = pi/4
T2 = np.matrix([[cos(theta2), -sin(theta2), x2],
                [sin(theta2), cos(theta2), y2],
                [0, 0, 1]])

# Robot 3
x3     = 0
y3     = 2
theta3 = -pi/6
T3 = np.matrix([[cos(theta3), -sin(theta3), x3],
                [sin(theta3), cos(theta3), y3],
                [0, 0, 1]])

# Robot 4
x4     = 0
y4     = -2
theta4 = -pi/3
T4 = np.matrix([[cos(theta4), -sin(theta4), x4],
                [sin(theta4), cos(theta4), y4],
                [0, 0, 1]])

x_fleet = 3
y_fleet = 5
theta_fleet = pi

T_fleet1 = np.matrix([[cos(theta_fleet), -sin(theta_fleet), x_fleet],
                        [sin(theta_fleet), cos(theta_fleet), y_fleet],
                        [0, 0, 1]])
fleet1 = Fleet(T_fleet1, "fleet1")
fleet1.add_robot(T1, "robot1")
fleet1.add_robot(T2, "robot2")
fleet1.add_robot(T3, "robot3")
fleet1.add_robot(T4, "robot4")
##########################

######### Fleet 2 ########


x_fleet = 10
y_fleet = 6
theta_fleet = pi

T_fleet2 = np.matrix([[cos(theta_fleet), -sin(theta_fleet), x_fleet],
                        [sin(theta_fleet), cos(theta_fleet), y_fleet],
                        [0, 0, 1]])

fleet2 = Fleet(T_fleet2, "fleet2")
# Let's use the same relative matrices for the sake of clarity
# The transformation is internally deep-copied
fleet2.add_robot(T1, "robot1")
fleet2.add_robot(T2, "robot2")
fleet2.add_robot(T3, "robot3")
fleet2.add_robot(T4, "robot4")
##########################

sim.add_fleet(fleet1)
sim.add_fleet(fleet2)

sim.run()
rospy.spin()
