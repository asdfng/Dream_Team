import time, json, urllib2
import timeit 
import os
import math
from a_star import AStar
import Orientation
a_star = AStar()

if __name__ == '__main__':
    try:
        last_orientation = talker('rTriangle','ball',0.0)
        end_orientation = talker('rTriangle','rCircle',last_orientation)
    except KeyboardInterrupt:
        a_star.motors(0,0)