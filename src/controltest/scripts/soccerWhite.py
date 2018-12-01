import time, json, urllib2
import timeit 
import os
import math
from a_star import AStar
from Orientation import grabber, displacement, point_orientation, run, orient, talker, fire, check



a_star = AStar() #Not sure that we need this line because it is defined in the orientation code

if __name__ == '__main__':
    try:
        last_orientation = talker('rTriangle','bTriangle',355.0)
    except KeyboardInterrupt:
        a_star.motors(0,0)