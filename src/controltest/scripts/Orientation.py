#!/usr/bin/env python
import time, json, urllib2
import timeit 
import os
import math
import RPi.GPIO as GPIO
from a_star import AStar


a_star = AStar()

def fire():
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(21,GPIO.OUT)

    GPIO.output(21,GPIO.HIGH)
    time.sleep(.1)
    GPIO.output(21,GPIO.LOW)

def grabber():
    response = urllib2.urlopen('http://192.168.137.1:8001/FieldData/GetData')
    source = response.read()
    data = json.loads(source.decode())
    locations = {'bSquare': {'X': data['Blue Team Data']['Square']['Object Center']['X'], 'Y': data['Blue Team Data']['Square']['Object Center']['Y']},
                'bCircle': {'X': data['Blue Team Data']['Circle']['Object Center']['X'], 'Y': data['Blue Team Data']['Circle']['Object Center']['Y']},
                'bTriangle': {'X': data['Blue Team Data']['Triangle']['Object Center']['X'], 'Y': data['Blue Team Data']['Triangle']['Object Center']['Y']},
                'rSquare': {'X': data['Red Team Data']['Square']['Object Center']['X'], 'Y': data['Red Team Data']['Square']['Object Center']['Y']},
                'rCircle': {'X': data['Red Team Data']['Circle']['Object Center']['X'], 'Y': data['Red Team Data']['Circle']['Object Center']['Y']},
                'rTriangle': {'X': data['Red Team Data']['Triangle']['Object Center']['X'], 'Y': data['Red Team Data']['Triangle']['Object Center']['Y']},
                'ball': {'X': data['Ball']['Object Center']['X'], 'Y': data['Ball']['Object Center']['Y']}}

    return locations

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

def check(comrade, target):
    locations = grabber()
    mGX = locations[target]['X']
    mGY = locations[target]['Y']
    mMeX = locations[comrade]['X']
    mMeY = locations[comrade]['Y']

    null1 ,mag = point_orientation(mMeX,mMeY,mGX,mGY)

    return mag

def orient(oLEncoder, oREncoder, compensated_orientation, previous_orientation):
    tAngle = previous_orientation
    previousA = previous_orientation
    while True:
        encoders = a_star.read_encoders()
        rEncoder = encoders[1]
        lEncoder = encoders[0]

        pRight = rEncoder - oREncoder
        pLeft  = lEncoder - oLEncoder

        oLEncoder = lEncoder
        oREncoder = rEncoder

        angle = displacement(pRight,pLeft) 
                              
        tAngle += angle
        cAngle = tAngle % 360
        

        lA = (cAngle - compensated_orientation) % 360
        rA = (compensated_orientation - cAngle) % 360

        if (((cAngle - 1) <= (compensated_orientation)) and ((compensated_orientation) <= (cAngle + 1))):
            a_star.motors(0,0) 
            break 
        elif ((lA - rA) > 25):
            a_star.motors(50,-50)
        elif ((lA - rA) < -25):
            a_star.motors(-50,50)
    return cAngle

def run(me, goal, oLEncoder, oREncoder, compensated_orientation, previous_orientation):
    current_angle = previous_orientation
    locations = grabber()
    if (goal == 'goal'):
        mGX = 339
        mGY = 128    
    else:
        mGX = locations[goal]['X']
        mGY = locations[goal]['Y']

    previousMeX = locations[me]['X']
    previousMeY = locations[me]['Y']

    if (goal == 'ball'):
        mark = 30
    else:
        mark = 40

    while True:
        
        spLeft = 100
        locations = grabber()

        rTX = locations['rTriangle']['X']
        rTY = locations['rTriangle']['Y']
        rCX = locations['rCircle']['X']
        rCY = locations['rCircle']['Y']
        rSX = locations['rSquare']['X']
        rSY = locations['rSquare']['Y']
        
        null0, checkT = point_orientation(previousMeX, previousMeY, rTX, rTY)
        null1, checkC = point_orientation(previousMeX, previousMeY, rCX, rCY)
        null2, checkS = point_orientation(previousMeX, previousMeY, rSX, rSY)

    
        shapes = {'rTriangle': float(checkT), 'rCircle':float(checkC), 'rSquare':float(checkS)}

        smallest = min(shapes, key = lambda x: shapes.get(x))

        mMeX = locations[smallest]['X']
        mMeY = locations[smallest]['Y']
        
        null3, mag = point_orientation(mMeX,mMeY,mGX,mGY)
        
        previousMeX = mMeX
        previousMeY = mMeY
        
        if (mag < mark):
            a_star.motors(0,0)
            break
        else:
            current_angle = orient(oLEncoder, oREncoder, compensated_orientation, previous_orientation)
            a_star.motors(100,100)
        
    return current_angle

def talker(me, goal, previous_orientation):
    
    angle_error_offset = 5.0
    encoders = a_star.read_encoders()
   
    oldright_encoder = encoders[1]
    oldleft_encoder = encoders[0]
   
    locations = grabber()

    if (goal == 'goal'):
        orientation_input, mag = point_orientation(locations[me]['X'],locations[me]['Y'],339,128)
    else:
        orientation_input, mag = point_orientation(locations[me]['X'],locations[me]['Y'],locations[goal]['X'],locations[goal]['Y'])

    compensated_orientation = (orientation_input - angle_error_offset) % 360

    last_angle = run(me, goal, oldleft_encoder,oldright_encoder,compensated_orientation, previous_orientation)
    return last_angle
   