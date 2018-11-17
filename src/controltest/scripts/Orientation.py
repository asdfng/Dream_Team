#!/usr/bin/env python
import time, json, urllib2
import timeit 
import os
import math
from a_star import AStar
from Grabber import grabber


def straight(speed):
    mLeft = speed
    sRight = speed
    sSlave = sRight
    encoders = a_star.read_encoders()
    oldencoderL = encoders[0]
    oldencoderR = encoders[1]
    tError = 0
    error = 0
    kp = 2
    ki = 1
    i = 0
    while (i < 10):
        if (i==0):
            a_star.motors(mLeft,sRight)
        encoders = a_star.read_encoders()
        encoderL = encoders[0]
        encoderR = encoders[1]
        dL = encoderL - oldencoderL
        dR = encoderR - oldencoderR
        error = dL - dR
        tError += error
        sSlave += error/kp #+ ki*tError
        oldencoderL = encoderL
        oldencoderR = encoderR
        a_star.motors(mLeft,sSlave)
        i = 1 + i
        time.sleep(0.05)

def displacement(right_encoder,left_encoder): #velocity: ft/s, position: ft
    theta_initial = 0.0
    theta_new_unbounded = 0.0
    pi = math.pi
    dist_between_wheels = 0.4791667
    right_wheel_rotations = right_encoder/float(1440)                  
    left_wheel_rotations = left_encoder/float(1440)                    
    right_displacement = right_wheel_rotations*float(2)*pi*.114829     
    left_displacement = left_wheel_rotations*float(2)*pi*.114829
    displacement_middle = (right_displacement + left_displacement)/float(2)
    alpha_left_turn_radians = (left_displacement - right_displacement)/dist_between_wheels
    alpha_left_turn_degrees = alpha_left_turn_radians * float(180)/pi
    theta_new_unbounded = theta_initial + alpha_left_turn_degrees 
    theta_new = theta_new_unbounded % 360                                   
    theta_initial = theta_new 
    return theta_new
                                                                
def point_orientation(our_point_x, our_point_y, desired_point_x, desired_point_y): #calculates the angle between the two points to figure out the correction
    pi = math.pi
    atan2 = math.atan2
    dist_x = float(desired_point_x) - float(our_point_x) #original orientation should be angle from gyro/encoder taken only once at the beginning of the process
    dist_y = float(desired_point_y) - float(our_point_y)
    mag = float(math.sqrt(math.pow(dist_x,2) + math.pow(dist_y,2)))
    angle = atan2(dist_y , dist_x) #can return [-pi,pi]
    angle_degrees = angle * float(180)/pi
    if angle_degrees >= 0:
        orientation_input_unbounded = angle_degrees + float(360)
    else:
        orientation_input_unbounded = float(360) + angle_degrees + float(360)
    orientation_input = orientation_input_unbounded % 360
    return orientation_input, mag

def run(me, goal):
    while True:
        mark = 1
        spLeft = 100
        locations = grabber()
        null1 , mag = point_orientation(locations[me]['X'],locations[me]['Y'],locations[goal]['X'],locations[goal]['Y'])
        print(mag)
        if (mag < mark):
            a_star.motors(0,0)
            break
        else:
            straight(spLeft)

def orient(oAngle, oLEncoder, oREncoder):
    
    encoders = a_star.read_encoders()
    rEncoder = encoders[1]
    lEncoder = encoders[0]

    pRight = rEncoder - oREncoder
    pLeft  = lEncoder - oLEncoder

    oLEncoder = lEncoder
    oREncoder = rEncoder

    angle_Encoder = displacement(passRight,passLeft) 
    dEncoder = angle - oAngle

    oldangle_Encoder = angle_Encoder                                     
    angle += dEncoder
    if (((angle - 1) <= (compensated_orientation)) and ((compensated_orientation) <= (angle + 1))):
        run(mag)
        a_star.motors(0,0) 
        break 
    elif ((orientation_input <= 360) and (orientation_input >= 180)):
        a_star.motors(-50,50)
    elif ((orientation_input <= 180) and (orientation_input >= 0)):
        a_star.motors(50,-50)


def talker(our_x, our_y, desired_x, desired_y,previous_orientation):
    
    a_star = AStar()

    encoders = a_star.read_encoders()
    oldright_encoder = encoders[1]
    oldleft_encoder = encoders[0]
    
    angle = previous_orientation
    total_displacement = 0.0
    
    locations = grabber()
    orientation_input, mag = point_orientation(locations[me]['X'],locations[me]['Y'],locations[goal]['X'],locations[goal]['Y'])
    
    if (orientation_input >= 180):
        angle_error_offset = -5.0
    else:
        angle_error_offset = 5.0
    if ((orientation_input < 5) or (orientation_input > 355)):
        compensated_orientation = 0
    else:
        compensated_orientation = orientation_input - angle_error_offset
    
