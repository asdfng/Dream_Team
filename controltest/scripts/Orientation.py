#!/usr/bin/env python
#import rospy
import time
import timeit 
import os
import math
from lms6 import LSM6
from a_star import AStar

#Initialize all objects
a_star = AStar()
imu = LSM6()
imu.enable()

a_star.motors(-25,25)

#Starting values and data for the imu
accelSensitivity = 0.061
accelRatio = 0.001 # Converting from milligrams to gram
gyroSensitivity = 0.035
sampleRate = .01 #100Hz

i = 0
angle = 0.0
angle_Gyro_unbounded = 0.0
total = 0.0

#Starting values for the encoders
theta_initial = 0.0
theta_new_unbounded = 0.0

def displacement(right_encoder,left_encoder): #velocity: ft/s, position: 
    
    global theta_initial
    global theta_new_unbounded

    pi = math.pi
    dist_between_wheels = 0.4791667
    
    #converts encoder counts to rotations
    right_wheel_rotations = right_encoder/float(1440)                  
    left_wheel_rotations = left_encoder/float(1440)

    #Prints the number of rotations, not needed for final iteration
    #print("rightrotations = %s rotations" % right_wheel_rotations)  
    #print("leftrotations = %s rotations" % left_wheel_rotations)

    #calculates displacement of right, left and center wheels                    
    right_displacement = right_wheel_rotations*float(2)*pi*.114829     
    left_displacement = left_wheel_rotations*float(2)*pi*.114829
    center_displacement = (right_displacement + left_displacement)/float(2)

    #calculates the change of the angle by a turn
    alpha_left_turn_radians = (right_displacement - left_displacement)/dist_between_wheels

    #converts to degrees
    alpha_left_turn_degrees = alpha_left_turn_radians * float(180)/pi

    #appends initial theta to new theta
    theta_new_unbounded = theta_initial + alpha_left_turn_degrees 
    theta_new = theta_new_unbounded % 360                                   
    theta_initial = theta_new 
    
    return theta_new
    #prints angle every 100ms, not needed for the final iteration
    #print("orientation = %s degrees" % theta_new)                               



def  talker():

    global angle, angle_Gyro_unbounded, total, i, sampleRate

    #Setup for the encoders
    encoders = a_star.read_encoders()
    oldright_encoder = encoders[1]
    oldleft_encoder = encoders[0]

    oldangle_Encoder = 0.0
    oldangle_Gyro = 0.0

    #pub = rospy.Publisher('/Enc_Degree', Orientation, queue_size=10)
    #rospy.init_node('Encoder_Orientation', anonymous=True)
    #rate = rospy.Rate(100)

    while True:

        Threshold = 0.125

        #Read the encoder and imu
        encoders = a_star.read_encoders()
        imu.read()
        #print(encoders[0], encoders[1])

        right_encoder = encoders[1]
        left_encoder = encoders[0]

        passRight = right_encoder - oldright_encoder
        passLeft = left_encoder - oldleft_encoder

        oldright_encoder = right_encoder 
        oldleft_encoder = left_encoder

        angle_Encoder = displacement(passRight,passLeft)
        print('Encoder: %s' % angle_Encoder)
    
        #Find the offset of the gyro and remove it
        while i<=10:
            
            imu.read()
            total += imu.g.z
            i += 10

        offsetGZ = total/10

        angle_Gyro_unbounded += (imu.g.z*gyroSensitivity-offsetGZ)*sampleRate
        angle_Gyro = angle_Gyro_unbounded % 360
        print('gyro: %s' % angle_Gyro)

        dGyro = angle_Gyro - oldangle_Gyro
        print('Delta gyro: %s' % dGyro)
        dEncoder = angle_Encoder - oldangle_Encoder
        print('Delta Encoder: %s' % dEncoder)

        oldangle_Encoder = angle_Encoder
        print('old encoder: %s' % oldangle_Encoder)
        oldangle_Gyro = angle_Gyro
        print('old gyro: %s' % oldangle_Gyro)

        if abs(dGyro - dEncoder) > Threshold:
            angle += dGyro
        else:
            angle += dEncoder
        

        #rate.sleep()
        print(angle)
        print(sampleRate)
        time.sleep(sampleRate)
        
        
        os.system('clear')

if __name__ == '__main__':
    try:
        talker()
    except KeyboardInterrupt:
        a_star.motors(0,0)
