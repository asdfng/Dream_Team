#!/usr/bin/env python
import rospy
import time 
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
sampleRate = .001 #100Hz

i = 0
angle = 0.0
angle_Gyro = 0.0
total = 0.0

#Starting values for the encoders
theta_initial = 0.0
theta_new = 0.0

def displacement(right_encoder,left_encoder): #velocity: ft/s, position: 
    
    global theta_initial
    global theta_new

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
    theta_new = theta_initial + alpha_left_turn_degrees                                     
    
    if  theta_new >= float(360):

        theta_new = theta_new - float(360)
        theta_initial = theta_new

    elif theta_new < float(0):

        theta_new = float(360) + theta_new
        theta_initial = theta_new

    else:
        
        #saves the new theta as the initial theta for next execution
        theta_initial = theta_new 
    
    return theta_new
    #prints angle every 100ms, not needed for the final iteration
    #print("orientation = %s degrees" % theta_new)                               



def  talker():

    global angle, angle_Gyro, total, i 

    #Setup for the encoders
    encoders = a_star.read_encoders()
    oldright_encoder = encoders[1]
    oldleft_encoder = encoders[0]

    oldangle_Encoder = 0.0
    oldangle_Gyro = 0.0

    pub = rospy.Publisher('/Enc_Degree', Orientation, queue_size=10)
    rospy.init_node('Encoder_Orientation', anonymous=True)
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        Threshold = 0.125

        #Read the encoders
        encoders = a_star.read_encoders()
        #print(encoders[0], encoders[1])

        right_encoder = encoders[1]
        left_encoder = encoders[0]

        passRight = right_encoder - oldright_encoder
        passLeft = left_encoder - oldleft_encoder

        oldright_encoder = right_encoder 
        oldleft_encoder = left_encoder

        angle_Encoder = displacement(passRight,passLeft)
    
        #Find the offset of the gyro and remove it
        while i<=10:
            total += imu.g.z
            i += 10

        offsetGZ = total/10

        angle_Gyro += (imu.g.z*gyroSensitivity-offsetGZ)*sampleRate

        dGyro = angle_Gyro - oldangle_Gyro
        dEncoder = angle_Encoder - oldangle_Encoder

        oldangle_Encoder = angle_Encoder 
        oldangle_Gyro = angle_Gyro

        if abs(dGyro - dEncoder) > Threshold:
            angle += dGyro
        else:
            angle += dEncoder
        

        rate.sleep()
        os.system('clear')

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
