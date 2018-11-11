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
imu = LSM6()
imu.enable()

#Starting values and data for the imu
accelSensitivity = 0.061
accelRatio = 0.001 # Converting from milligrams to gram
gyroSensitivity = 0.035
sampleRate = .1 #100Hz

i = 0
angle = 0.0
angle_Gyro_unbounded = 0.0
total = 0.0

#Starting values for the encoders
theta_initial = 0.0
theta_new_unbounded = 0.0
def straight(speed):
    #Store the master speed into values
    mLeft = speed
    sRight = speed
    sSlave = sRight
    encoders = a_star.read_encoders()
    oldencoderL = encoders[0]
    oldencoderR = encoders[1]
    tError = 0
    error = 0
    kp = 2  #proportional constant
    ki = 1  #integral constant
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
        print(sSlave)
        print(mLeft)
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

#This code will find the distance it needs to move in order to get to the point
def run(mag):

    while True:
        mark = 1
        spLeft = 100

        mRSX, mRSY, mBX, mBY = fetch_coordinates()
        nn, mag = point_orientation(mRSX,mRSX,mBX,mBX)

        if (mag < mark):
            a_star.motors(0,0)
            break
        else:
            straight(spLeft)

def  talker():
    global angle, angle_Gyro_unbounded, total, i, sampleRate
    #Setup for the encoders
    encoders = a_star.read_encoders()
    oldright_encoder = encoders[1]
    oldleft_encoder = encoders[0]
    oldangle_Encoder = 0.0
    oldangle_Gyro = 0.0
    total_displacement = 0.0
    mRSX, mRSY, mBX, mBY = fetch_coordinates()
    orientation_input, mag = point_orientation(mRSX,mRSY,mBX,mBY)
    while True:
        start_time = timeit.default_timer()

        Threshold = 0.125

        #Read the encoder and imu
        encoders = a_star.read_encoders()  
        imu.read()

        right_encoder = encoders[1]
        left_encoder = encoders[0]

        passRight = right_encoder - oldright_encoder
        passLeft = left_encoder - oldleft_encoder

        oldright_encoder = right_encoder 
        oldleft_encoder = left_encoder
        angle_Encoder, center_displacement, right_displacement, left_displacement = displacement(passRight,passLeft) 
        
    
        #Find the offset of the gyro and remove it
        while i<=10:
            
            imu.read()
            total += imu.g.z
            i += 10

        offsetGZ = total/10

        angle_Gyro_unbounded += (imu.g.z*gyroSensitivity-offsetGZ)*sampleRate
        angle_Gyro = angle_Gyro_unbounded % 360
        dGyro = angle_Gyro - oldangle_Gyro
        dEncoder = angle_Encoder - oldangle_Encoder
        total_displacement += center_displacement
        oldangle_Encoder = angle_Encoder
        oldangle_Gyro = angle_Gyro                                      
        #if abs(dGyro - dEncoder) < Threshold:
            #angle += dGyro
        #else:
        angle += dEncoder
        print("orientation = %s" % angle)
        print("displacement = %s" % total_displacement)
        #print(red_square_x)
        #print(type(red_square_x))
        #print(red_square_y)
        #print(ball_x)
        #print(ball_y)
        #print(mRSX)
        #print(mRSY)
        #print(mBX)
        #print(mBY)
        print('orientation_input = %s' % orientation_input)
        if (((angle - 1) <= orientation_input) and (orientation_input <= (angle + 1))): #current orientation should just be angle of encoder or gyro
            run(mag)
            a_star.motors(0,0)  
        else:
            a_star.motors(50,-50)
        
        sampleRate = timeit.default_timer() - start_time
        os.system("clear")




if __name__ == '__main__':
    try:
        talker()
    except KeyboardInterrupt:
        a_star.motors(0,0)