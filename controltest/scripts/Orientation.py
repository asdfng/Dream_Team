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
sampleRate = .1 #100Hz

i = 0
angle = 0.0
angle_Gyro_unbounded = 0.0
total = 0.0

#Starting values for the encoders
theta_initial = 0.0
theta_new_unbounded = 0.0

def displacement(right_encoder,left_encoder): #velocity: ft/s, position: ft
    
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
                                                                
def point_orientation(our_point_x, our_point_y, desired_point_x, desired_point_y, original_orientation): #calculates the angle between the two points to figure out the correction
    pi = math.pi
    atan = math.atan
    dist_x = float(desired_point_x) - float(our_point_x) #original orientation should be angle from gyro/encoder taken only once at the beginning of the process
    dist_y = float(desired_point_y) - float(our_point_y)
    mag = math.sqrt((dist_x)** float(2) + (dist_y)**float(2))
    angle = atan((dist_y) / (dist_x))
    angle_degrees = angle * float(180)/pi 
    orientation_input_unbounded = angle_degrees + float(360) - float(original_orientation)
    orientation_input = orientation_input_unbounded % 360
    return orientation_input
   
#def __init__(self)
    #pub = rospy.Publisher('/Enc_Degree', Orientation, queue_size=10)
    #rospy.init_node('Encoder_Orientation', anonymous=True)
    #rate = rospy.Rate(100)
    #rospy.Suscriber('mapper/new_data/red/triangle',Rover,queue_size=10)
    #rospy.Suscriber('mapper/new_data/red/square',Rover,queue_size=10)
    #rospy.Suscriber('mapper/new_data/red/circle',Rover,queue_size=10)
    #rospy.Suscriber('mapper/new_data/ball',Rover,queue_size=10)
#callback?
    #playerRCX=player_rc.center.x
    #playerRCY=player_rc.center.y
    #playerRSX=player_rs.center.x
    #playerRSY=player_rs.center.y
    #playerRTX=player_rt.center.x
    #playerRTY=player_rt.center.y
    #ballX=ball.x
    #ballY=ball.y


def  talker():

    global angle, angle_Gyro_unbounded, total, i, sampleRate

    #Setup for the encoders
    encoders = a_star.read_encoders()
    oldright_encoder = encoders[1]
    oldleft_encoder = encoders[0]

    oldangle_Encoder = 0.0
    oldangle_Gyro = 0.0

    orientation_input = point_orientation(0,0,3,3,angle) #dummy coordinates for now

    while True:
        start_time = timeit.default_timer()

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
        print('orientation_input = %s' % orientation_input)
        if abs(dGyro - dEncoder) < Threshold:
            angle += dGyro
        else:
            angle += dEncoder
        
        #if angle - 5 >= orientation_input  or angle + 5 <= orientation_input: #current orientation should just be angle of encoder or gyro
            #a_star.motors(0,0)
        #else:
            #a_star.motors(-25,25)
        
        #rate.sleep()
        print(angle)
        print(sampleRate)
        time.sleep(0.01) #Make sure this is equal to the output of the sample rate, DO NOT USE THE VARIABLE
        
        sampleRate = timeit.default_timer() - start_time

        os.system('clear')


if __name__ == '__main__':
    try:
        talker()
    except KeyboardInterrupt:
        a_star.motors(0,0)