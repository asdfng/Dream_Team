#!/usr/bin/env python
#import rospy
import time
import timeit 
import os
import math
from lms6 import LSM6
from a_star import AStar
a_star = AStar()
def point_orientation(our_point_x, our_point_y, desired_point_x, desired_point_y, original_angle): #calculates the angle between the two points to figure out the correction
    dist_x = our_point_x - desired_point_x 
    dist_y = our_point_y - desired_point_y
    mag = math.sqrt(dist_x** 2 + dist_y**2)
    angle = math.atan(dist_y / dist_x)
    angle_degrees = angle * 180/(math.pi)
    print('angle = %s' % angle_degrees)
    orientation_input = angle_degrees + original_angle
    return orientation_input

def orient_robot(orientation_input, current_orientation):
    while True:
        if current_orientation - 5 >= orientation_input  or current_orientation + 5 <= orientation_input:
            a_star.motors(0,0)
            break
        else:
            a_star.motors(-25,25)