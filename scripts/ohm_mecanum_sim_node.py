#!/usr/bin/env python3

# ------------------------------------------------------------
# Author:      Stefan May
# Date:        20.4.2020
# Description: Pygame-based robot simulator application
# ------------------------------------------------------------

import pygame
import rospy
from ohm_mecanum_simulator import Ohm_Mecanum_Simulator

import numpy as np
from math import cos, sin, pi, sqrt, acos

pygame.init()

# Screen size
size = width, height = 1600, 900

# Drawing surface
surface = pygame.display.set_mode(size, pygame.HWSURFACE | pygame.DOUBLEBUF)

sim = Ohm_Mecanum_Simulator(surface, "ohm_mecanum_sim", "Ohm Mecanum Simulator")

# Robot 1
x1     = 2
y1     = 3
theta1 = 0

T1 = np.matrix([[cos(theta1), -sin(theta1), x1],
                [sin(theta1), cos(theta1), y1],
                [0, 0, 1]])

# Robot 2
x2     = 4
y2     = 3
theta2 = 0

T2 = np.matrix([[cos(theta2), -sin(theta2), x2],
                [sin(theta2), cos(theta2), y2],
                [0, 0, 1]])

# Robot 3
x3     = 3
y3     = 4
theta3 = 0

T3 = np.matrix([[cos(theta3), -sin(theta3), x3],
                [sin(theta3), cos(theta3), y3],
                [0, 0, 1]])

# Common kinematic center
x_center = 3
y_center = 3
theta_center = 0

T_center = np.matrix([[cos(theta_center), -sin(theta_center), x_center],
                        [sin(theta_center), cos(theta_center), y_center],
                        [0, 0, 1]])

sim.spawn_robot(T1, T_center, "robot1")
sim.spawn_robot(T2, T_center, "robot2")
sim.spawn_robot(T3, T_center, "robot3")

border = 5
sim.add_rectangle_pixelcoords([border, border], [width-border, height-border])

sim.run()
rospy.spin()
