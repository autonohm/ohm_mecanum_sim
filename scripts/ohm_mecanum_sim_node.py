#!/usr/bin/env python3

# ------------------------------------------------------------
# Author:      Stefan May
# Date:        20.4.2020
# Updated:     24.09.2024 by Dong Wang
# Updated:     16.10.2024 by Marco Masannek
# Description: Pygame-based robot simulator application
# ------------------------------------------------------------

import pygame
import rospy
import os
from ohm_mecanum_simulator import Ohm_Mecanum_Simulator

MAP_NAME = "mrcn_map.png"
map_img_path = os.path.join(os.path.dirname(__file__), "../images/" + MAP_NAME)

pygame.init()

# Screen size
size = width, height = 900, 900
# Drawing surface
surface = pygame.display.set_mode(size, pygame.HWSURFACE | pygame.DOUBLEBUF)

sim = Ohm_Mecanum_Simulator(surface, map_img_path, "ohm_mecanum_sim", "Ohm Mecanum Simulator")
# the intial position of the robot should be offset in the odometry publisher
sim.spawn_robot(2, 2, 0, "kobuki")

# sim.spawn_robot(5, 7, 0, "robot2")
# border = 5
# sim.add_rectangle_pixelcoords([border, border], [width-border, height-border])
# border = 300
# sim.add_rectangle_pixelcoords([border, border], [width-border, height-border])

sim.run()
rospy.spin()
