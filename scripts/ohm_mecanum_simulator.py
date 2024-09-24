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

class Ohm_Mecanum_Simulator:

    def __init__(self, surface, map, rosname, windowtitle):
        self._surface = surface
        self._map = pygame.transform.scale(pygame.image.load(map), self._surface.get_size())
        self._meter_to_pixel = 100
        self._robots = []
        self._line_segment_obstacles = []
        self._verbose = True
        rospy.init_node(rosname, anonymous=True)
        pygame.display.set_caption(windowtitle)
    
    def __del__(self):
        pass

    def service_callback_spawn(self, req):
        self.spawn_robot(req.x, req.y, req.theta, req.name)
        response = SpawnResponse(req.x, req.y, req.theta, req.name)
        return response

    def service_callback_kill(self, req):
        self.kill_robot(req.name)
        response = KillResponse(True)
        return response

    def service_callback_verbose(self, req):
        self._verbose = req.data
        if(self._verbose):
            msg = "Verbosity increased"
        else:
            msg = "Verbosity decreased"
        return SetBoolResponse(True, msg)

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

    def run(self):
        # bg_color = (64, 64, 255)
        rospy.Service('/spawn', Spawn, self.service_callback_spawn)
        rospy.Service('/kill', Kill, self.service_callback_kill)
        rospy.Service('/verbose', SetBool, self.service_callback_verbose)
        rate = rospy.Rate(50)

        clock = pygame.time.Clock()
        clock.tick(360)
        mouse_focused = False
        

        while 1:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.exit_simulation()
                # elif pygame.mouse.get_focused():
                #     mouse_focused = True
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_c and pygame.key.get_mods() & pygame.KMOD_CTRL:
                        self.exit_simulation()

            # self._surface.fill(bg_color)
            
            # Draw obstacles using the map image
            self._surface.blit(self._map, (0, 0))
            # Draw obstacles using line segments
            # for obstacle in self._line_segment_obstacles:
            #     pixel_segment_start = self.transform_to_pixelcoords(obstacle[0])
            #     pixel_segment_end = self.transform_to_pixelcoords(obstacle[1])
            #     pygame.draw.line(self._surface, pygame.Color(0, 0, 0), pixel_segment_start, pixel_segment_end, 3)

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
                robot_pose = pixel_robot[0], pixel_robot[1] , r.get_heading()
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
                # sensor sensing
                dist_to_obstacles = r.LiDAR_sensing(robot_pose,self._surface)
                r.publish_LiDAR(dist_to_obstacles)
                # for obstacle in self._line_segment_obstacles:
                #     dist_to_obstacles = r.get_distance_to_line_obstacle(obstacle[0], obstacle[1], dist_to_obstacles)

                # r.publish_tof(dist_to_obstacles)

                r.release_lock()

                min_dist = 9999
                for i in range(0, len(dist_to_obstacles)):
                    if(dist_to_obstacles[i]<min_dist and dist_to_obstacles[i]>0):
                        min_dist = dist_to_obstacles[i];
                #0.2 + 0.02 to avoid collision
                if(min_dist<(0.22)):
                    r.reset_pose()
                elif (r._coords[0] < 0 or r._coords[1] < 0 or r._coords[0] > self._surface.get_width()/self._meter_to_pixel or r._coords[1] > self._surface.get_height()/self._meter_to_pixel):
                    r.reset_pose()

                # Draw ToF beams
                # pos_hitpoint = r.get_hit_tof(dist_to_obstacles)
                # for i in range(0,r.get_tof_count()):
                #     pixel_sensor = self.transform_to_pixelcoords(pos_sensor[i])
                #     pixel_hitpoint = self.transform_to_pixelcoords(pos_hitpoint[i])
                #     pygame.draw.line(self._surface, pygame.Color(255, 0, 0), pixel_sensor, pixel_hitpoint)          

            pygame.display.update()

            rate.sleep()
