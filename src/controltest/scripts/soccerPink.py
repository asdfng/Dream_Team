import time, json, urllib2
import timeit 
import os
import math
from a_star import AStar
from Orientation import grabber, displacement, point_orientation, run, orient, talker, fire, check

a_star = AStar()

def execute()
    orientation = 0.0
    while True:
        orientation = talker('rCircle','ball',orientation)
        orientation = talker ('rCircle','goal',orientation)
        if check('rCircle','ball') > 40:
            continue
        else:
            fire()
            break
    

if __name__ == '__main__':
    try:
        execute()
    except KeyboardInterrupt:
        a_star.motors(0,0)