#!/usr/bin/env python
#import rospy
import time, json, urllib2
import timeit 
import os
import math
from lms6 import LSM6
from a_star import AStar

#Initialize all objects
a_star = AStar()
angle = 0.0
angle_Gyro_unbounded = 0.0
total = 0.0
theta_initial = 0.0
theta_new_unbounded = 0.0

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
    global theta_initial
    global theta_new_unbounded
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
    return theta_new, displacement_middle, right_displacement, left_displacement
                                                                
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

def fetch_coordinates():
    response = urllib2.urlopen('http://192.168.137.1:8001/FieldData/GetData')
    source = response.read()
    data = json.loads(source.decode())
    red_square_x = data['Red Team Data']['Square']['Object Center']['X']
    red_square_y = data['Red Team Data']['Square']['Object Center']['Y']
    ball_x = data['Ball']['Object Center']['X']
    ball_y = data['Ball']['Object Center']['Y']
    mRSX = float(red_square_x - 12)*(float(8/float(394-12)))
    mRSY = float(red_square_y - 31)*(float(4/float(221-31)))
    mBX = float(ball_x - 12)*(float(8/float(394-12)))
    mBY = float(ball_y - 31)*(float(4/float(221-31)))
    return mRSX, mRSY, mBX, mBY

def run(mag):
    while True:
        mark = 1
        spLeft = 100
        mRSX, mRSY, mBX, mBY = fetch_coordinates()
        unused_angle, mag = point_orientation(mRSX,mRSX,mBX,mBX)
        print(mag)
        if (mag < mark):
            a_star.motors(0,0)
            break
        else:
            straight(spLeft)

def  talker():
    global angle, angle_Gyro_unbounded, total, i, sampleRate
    encoders = a_star.read_encoders()
    oldright_encoder = encoders[1]
    oldleft_encoder = encoders[0]
    oldangle_Encoder = 0.0
    total_displacement = 0.0
    mRSX, mRSY, mBX, mBY = fetch_coordinates()
    orientation_input, mag = point_orientation(mRSX,mRSY,mBX,mBY)
    while True:
        encoders = a_star.read_encoders()  
        right_encoder = encoders[1]
        left_encoder = encoders[0]
        passRight = right_encoder - oldright_encoder
        passLeft = left_encoder - oldleft_encoder
        oldright_encoder = right_encoder 
        oldleft_encoder = left_encoder
        angle_Encoder, center_displacement, right_displacement, left_displacement = displacement(passRight,passLeft) 
        dEncoder = angle_Encoder - oldangle_Encoder
        total_displacement += center_displacement
        oldangle_Encoder = angle_Encoder                                     
        angle += dEncoder
        print("orientation = %s" % angle)
        print("displacement = %s" % total_displacement)
        print('orientation_input = %s' % orientation_input)
        if (((angle - 1) <= orientation_input - 5) and (orientation_input - 5 <= (angle + 1))): #current orientation should just be angle of encoder or gyro
            run(mag)
            a_star.motors(0,0) 
            break 
        elif ((orientation_input <= 360) and (orientation_input >= 180)):
            a_star.motors(-50,50)
        elif ((orientation_input <= 180) and (orientation_input >= 0)):
            a_star.motors(50,-50)
        os.system("clear")

if __name__ == '__main__':
    try:
        talker()
    except KeyboardInterrupt:
        a_star.motors(0,0)
