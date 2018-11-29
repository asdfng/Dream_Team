import time, json, urllib2
import timeit 
import os
import math
from a_star import AStar
from Orientation import grabber, displacement, point_orientation, run, orient, talker, fire, check

a_star = AStar()

def execute():
    last_orientation = talker('rCircle','rTriangle',0.0)
    distance = check('rTriangle','bCircle')
    while (distance > 50):
        distance = check('rTriangle','bCircle')
    end_orientation = talker('rCircle','ball',last_orientation)
    # a_star.motors(50,50)
    # time.sleep(1)
    # a_star.motors(0,0)
    nn = talker('rCircle','goal',end_orientation)
    # angle_error_offset = 5.0
    # encoders = a_star.read_encoders()
   
    # oldright_encoder = encoders[1]
    # oldleft_encoder = encoders[0]
   
    # locations = grabber()

    # orientation_input, mag = point_orientation(locations['rCircle']['X'],locations['rCircle']['Y'],339,128)

    # compensated_orientation = (orientation_input - angle_error_offset) % 360
    # orient(oldleft_encoder,oldright_encoder,compensated_orientation, end_orientation)
    
    fire()
    

if __name__ == '__main__':
    try:
        execute()
    except KeyboardInterrupt:
        a_star.motors(0,0)