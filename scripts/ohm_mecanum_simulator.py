#!/usr/bin/env python3

# ------------------------------------------------------------
# Author:      Stefan May
# Date:        20.4.2020
# Description: Pygame-based robot simulator with ROS interface
# ------------------------------------------------------------

import rospy
import pygame
import sys
from std_srvs.srv import SetBool, SetBoolResponse
from robot import Robot
from ohm_mecanum_sim.srv import Spawn, Kill, SpawnRequest, SpawnResponse, KillRequest, KillResponse
from std_msgs.msg import String
import numpy as np

class Ohm_Mecanum_Simulator:

    def __init__(self, surface, rosname, windowtitle):
        self._surface = surface
        self._meter_to_pixel = 100
        self._fleets = []

        rospy.init_node(rosname, anonymous=True)
        pygame.display.set_caption(windowtitle)
    
    def __del__(self):
        pass

    def add_fleet(self, fleet):
        self._fleets.append(fleet)

    def transform_to_pixelcoords(self, coords):
        pixelcoords  = [ coords[0] * self._meter_to_pixel,
                        (self._surface.get_height() - coords[1] * self._meter_to_pixel) ]
        return pixelcoords

    def transform_to_robotcoords(self, coords):
        pixelcoords  = [ coords[0] / self._meter_to_pixel,
                         (-coords[1] + self._surface.get_height()) / self._meter_to_pixel]
        return pixelcoords

    def exit_simulation(self):
        print("Exit simulation")
        for f in self._fleets:
            robots = f.get_robots()
            for r in robots:
                r.stop()
                del r
        sys.exit()

    def run(self):
        bg_color = (64, 64, 255)

        rate = rospy.Rate(50)

        clock = pygame.time.Clock()
        clock.tick(360)

        while 1:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.exit_simulation()
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_c and pygame.key.get_mods() & pygame.KMOD_CTRL:
                        self.exit_simulation()

            self._surface.fill(bg_color)
            
            # Convert robot coordinates for displaying all entities in pixel coordinates
            for f in self._fleets:
                robots = f.get_robots()

                for r in robots:
                    r.acquire_lock()
                    # Draw robot symbol
                    coords      = r.get_coords()
                    pixel_robot = self.transform_to_pixelcoords(coords)
                    rect        = r.get_rect()
                    rect.center = pixel_robot
                    rect.move(pixel_robot)
                    self._surface.blit(r.get_image(), rect)
                    r.release_lock()

            pygame.display.update()

            rate.sleep()
