import time, json, urllib2
import timeit 
import os
import math
from a_star import AStar
from Orientation import grabber, displacement, point_orientation, run, orient, talker, fire

a_star = AStar()

def execute():
    last_orientation = talker('rSquare','rTriangle',0.0)
    distance = check('rTriangle','bCircle')
    while (distance > 50):
        distance = check('rTriangle','bCircle')
    end_orientation = talker('rSquare','ball',last_orientation)
    nn = talker('rSquare','goal',end_orientation)
    fire()
    

if __name__ == '__main__':
    try:
        execute()
    except KeyboardInterrupt:
        a_star.motors(0,0)