#!/usr/bin/env python3

# ------------------------------------------------------------
# Author:      Stefan May
# Date:        20.4.2020
# Description: Pygame-based robot simulator application
# ------------------------------------------------------------

import pygame
import rospy
from ohm_mecanum_simulator import Ohm_Mecanum_Simulator

pygame.init()

# Screen size
size = width, height = 1600, 900

# Drawing surface
surface = pygame.display.set_mode(size, pygame.HWSURFACE | pygame.DOUBLEBUF)

sim = Ohm_Mecanum_Simulator(surface, "ohm_mecanum_sim", "Ohm Mecanum Simulator")

sim.spawn_robot(2, 2, 0, "robot1")
# sim.spawn_robot(5, 7, 0, "robot2")

border = 5
sim.add_rectangle_pixelcoords([border, border], [width-border, height-border])
border = 300
sim.add_rectangle_pixelcoords([border, border], [width-border, height-border])

sim.run()
rospy.spin()
