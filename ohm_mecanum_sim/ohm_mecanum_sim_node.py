#!/usr/bin/env python3

# ------------------------------------------------------------
# Author:      Stefan May
# Date:        01.05.2024
# Description: Pygame-based robot simulator application for ROS2
# ------------------------------------------------------------

import pygame
import rclpy
from rclpy.node import Node
from ohm_mecanum_sim.ohm_mecanum_simulator import Ohm_Mecanum_Simulator
from rclpy.executors import SingleThreadedExecutor

from std_msgs.msg import String

def main(args=None):
    pygame.init()
    rclpy.init(args=args)

    # Screen size
    size = width, height = 1600, 900

    # Drawing surface
    surface = pygame.display.set_mode(size, pygame.HWSURFACE | pygame.DOUBLEBUF)

    # Configure simulator and arena
    sim = Ohm_Mecanum_Simulator(surface, "ohm_mecanum_sim", "Ohm Mecanum Simulator")
    border = 5
    sim.add_rectangle_pixelcoords([border, border], [width-border, height-border])
    border = 300
    sim.add_rectangle_pixelcoords([border, border], [width-border, height-border])
    sim.start_scheduler()
    
    executor = SingleThreadedExecutor()
    executor.add_node(sim)
    executor.add_node(sim.spawn_robot(2, 2, 0, "robot1"))
    # executor.add_node(sim.spawn_robot(5, 7, 0, "robot2"))
    executor.spin()

    sim.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
