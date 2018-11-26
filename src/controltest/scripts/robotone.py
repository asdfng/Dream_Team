import time, json, urllib2
import timeit 
import os
import math
from a_star import AStar
from Orientation import straight, grabber, displacement, point_orientation, run, orient, talker, fire



a_star = AStar() #Not sure that we need this line because it is defined in the orientation code

if __name__ == '__main__':
    try:
        last_orientation = talker('rTriangle','ball',0.0)
        i = 0
        while i < 10:
            a_star.motors(-50,-50)
            i = i + 10
        a_star.motors(0,0)
        end_orientation = talker('rTriangle','rCircle',last_orientation)
    except KeyboardInterrupt:
        a_star.motors(0,0)