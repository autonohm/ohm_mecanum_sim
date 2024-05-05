#!/usr/bin/env python3

# ------------------------------------------------------------
# Author:      Stefan May
# Date:        01.05.2024
# Description: Pygame-based robot simulator with ROS2 interface
# ------------------------------------------------------------

import pygame
import sys

import rclpy
from rclpy.node import Node

from ohm_mecanum_sim.robot import Robot

class Ohm_Mecanum_Simulator(Node):

    def __init__(self, surface, rosname, windowtitle):
        super().__init__(rosname)
        self._surface = surface
        self._meter_to_pixel = 100
        self._robots = []
        self._line_segment_obstacles = []
        self._verbose = False
        timer_period = 0.05
        pygame.display.set_caption(windowtitle)

    def __del__(self):
        pass

    def start_scheduler(self):
        clock = pygame.time.Clock()
        clock.tick(360)
        timer_period = 0.02
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
        bg_color = (64, 64, 255)
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.exit_simulation()
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_c and pygame.key.get_mods() & pygame.KMOD_CTRL:
                    self.exit_simulation()
        self._surface.fill(bg_color)

        # Draw obstacles
        for obstacle in self._line_segment_obstacles:
            pixel_segment_start = self.transform_to_pixelcoords(obstacle[0])
            pixel_segment_end = self.transform_to_pixelcoords(obstacle[1])
            pygame.draw.line(self._surface, pygame.Color(0, 0, 0), pixel_segment_start, pixel_segment_end, 3)

        # Convert robot coordinates for displaying all entities in pixel coordinates
        for r in self._robots:

            r.acquire_lock()
            # Draw robot symbol
            coords      = r.get_coords()
            pixel_robot = self.transform_to_pixelcoords(coords)
            rect        = r.get_rect()
            rect.center = pixel_robot
            rect.move(pixel_robot)
            self._surface.blit(r.get_image(), rect)

            pos_sensor = r.get_pos_tof()
            pos_hitpoint = r.get_far_tof()

            # Determine distances to other robots
            dist_to_obstacles  = []
            for obstacle in self._robots:
                if(obstacle != r):
                    obstacle_coords = obstacle.get_coords()
                    dist_to_obstacles = r.get_distance_to_circular_obstacle(obstacle_coords, obstacle.get_obstacle_radius(),  dist_to_obstacles)

                    # Draw circular radius of obstacle
                    if(self._verbose):
                        pixel_obstacle = self.transform_to_pixelcoords(obstacle_coords)
                        obstacle_rect = obstacle.get_rect()
                        obstacle_rect.center = pixel_obstacle
                        obstacle_rect.move(pixel_obstacle)
                        pygame.draw.circle(self._surface, (255, 0, 0), (int(pixel_obstacle[0]), int(pixel_obstacle[1])), int(obstacle.get_obstacle_radius()*self._meter_to_pixel), 1)

            # Determine distances to line segments
            for obstacle in self._line_segment_obstacles:
                dist_to_obstacles = r.get_distance_to_line_obstacle(obstacle[0], obstacle[1], dist_to_obstacles)

            #r.publish_tof(dist_to_obstacles)

            r.release_lock()

        pygame.display.update()
    
    def spawn_robot(self, x, y, theta, name):
        self._robots.append(Robot(x, y, theta, name))

    def kill_robot(self, name):
        for r in self._robots:
            if(r._name == name):
                r.stop()
                self._robots.remove(r)

    def add_line_segment_pixelcoords(self, coords1, coords2):
        line_segment = (self.transform_to_robotcoords(coords1), self.transform_to_robotcoords(coords2))
        self.add_line_segment_obstacle(line_segment)

    def add_rectangle_pixelcoords(self, coords1, coords2):
        line_segment = (self.transform_to_robotcoords([coords1[0], coords1[1]]), self.transform_to_robotcoords([coords1[0], coords2[1]]))
        self.add_line_segment_obstacle(line_segment)
        line_segment = (self.transform_to_robotcoords([coords1[0], coords2[1]]), self.transform_to_robotcoords([coords2[0], coords2[1]]))
        self.add_line_segment_obstacle(line_segment)
        line_segment = (self.transform_to_robotcoords([coords2[0], coords2[1]]), self.transform_to_robotcoords([coords2[0], coords1[1]]))
        self.add_line_segment_obstacle(line_segment)
        line_segment = (self.transform_to_robotcoords([coords2[0], coords1[1]]), self.transform_to_robotcoords([coords1[0], coords1[1]]))
        self.add_line_segment_obstacle(line_segment)


    def add_line_segment_obstacle(self, line_segment):
        self._line_segment_obstacles.append(line_segment)

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
        for r in self._robots:
            r.stop()
            del r
        sys.exit()
