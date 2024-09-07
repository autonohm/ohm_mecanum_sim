import pygame
import math
import random
import rospy
from slam_demo import SLAM_Demo

slam_demo = SLAM_Demo("slam_demo", "SLAM Demo")
slam_demo.run()
rospy.spin()




