#!/usr/bin/env python3

# ------------------------------------------------------------
# Author:      Stefan May
# Date:        1.5.2024
# Description: Pygame-based robot simulator application for ROS2
# ------------------------------------------------------------

import pygame
import rclpy
#from ohm_mecanum_simulator import Ohm_Mecanum_Simulator

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    pygame.init()

    # Screen size
    size = width, height = 1600, 900

    # Drawing surface
    surface = pygame.display.set_mode(size, pygame.HWSURFACE | pygame.DOUBLEBUF)

    #sim = Ohm_Mecanum_Simulator(surface, "ohm_mecanum_sim", "Ohm Mecanum Simulator")

    #sim.spawn_robot(2, 2, 0, "robot1")
    # sim.spawn_robot(5, 7, 0, "robot2")

    #border = 5
    #sim.add_rectangle_pixelcoords([border, border], [width-border, height-border])
    #border = 300
    #sim.add_rectangle_pixelcoords([border, border], [width-border, height-border])

    #sim.run()

    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
