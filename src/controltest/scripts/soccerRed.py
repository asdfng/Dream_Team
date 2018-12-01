import time, json, urllib2
import timeit 
import os
import math
from a_star import AStar
from Orientation import grabber, displacement, point_orientation, run, orient, talker, fire, check

a_star = AStar()

def execute():
    last_orientation = 0.0
    while True:
        last_orientation = talker('rSquare','bCircle',last_orientation)
    

if __name__ == '__main__':
    try:
        execute()
    except KeyboardInterrupt:
        a_star.motors(0,0)