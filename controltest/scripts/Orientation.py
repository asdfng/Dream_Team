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
    
    return theta_new, center_displacement, right_displacement, left_displacement
                                                                
def point_orientation(our_point_x, our_point_y, desired_point_x, desired_point_y, original_orientation): #calculates the angle between the two points to figure out the correction
    pi = math.pi
    atan2 = math.atan2
    dist_x = float(desired_point_x) - float(our_point_x) #original orientation should be angle from gyro/encoder taken only once at the beginning of the process
    dist_y = float(desired_point_y) - float(our_point_y)
    mag = float(math.sqrt(math.pow(dist_x,2) + math.pow(dist_y,2)))
    angle = atan2(dist_y , dist_x) #can return [-pi,pi]
    angle_degrees = angle * float(180)/pi
    if angle_degrees >= 0:
        orientation_input_unbounded = angle_degrees + float(360) - float(original_orientation)
    else:
        orientation_input_unbounded = float(360) + angle_degrees + float(360) - float(original_orientation)
    orientation_input = orientation_input_unbounded % 360
    return orientation_input, angle_degrees, mag

#This code will find the distance it needs to move in order to get to the point
def run(mag):
    #Read the Encoders
    encoders_run = a_star.read_encoders()
    right_encoder2 = encoders_run[1]
    left_encoder2 = encoders_run[0]

    #Find the starting displacement since the last time the code ran
    angle_Encoder2, center_displacement2, right_displacement2, left_displacement2 = displacement(right_encoder2,left_encoder2)

    oldDis = right_displacement2 - left_displacement2

    #Initialize the displacement to be zero at the start of each run
    displacement_current = 0.0


    #Set the motors
    a_star.motors(100,100)

    while displacement_current < mag:

        #Read the Encoders
        encoders_run = a_star.read_encoders()
        right_encoder2 = encoders_run[1]
        left_encoder2 = encoders_run[0]

        #Find the displacement since the last time the code ran
        angle_Encoder2, center_displacement2, right_displacement2, left_displacement2 = displacement(right_encoder2,left_encoder2)

        #Get the correct center displacement
        CurrentDis = right_displacement2 - left_displacement2

        #find the change in the displacement
        dDis = abs(CurrentDis - oldDis)

        oldDis = CurrentDis
        
        #Add the displacement to the current displacement
        displacement_current += abs(dDis)
        

        print('center_displacement = %s' % displacement_current)
        print('change in displacement = %s' % dDis)
        print('displacement = %s' % CurrentDis )
        print('right_displacement = %s' % right_displacement2)
        print('left_displacement = %s' % left_displacement2)
        print('mag = %s' % mag)
        os.system('clear')


def  talker():

    global angle, angle_Gyro_unbounded, total, i, sampleRate
    a_star.motors(-50,50)

    #Setup for the encoders
    encoders = a_star.read_encoders()
    oldright_encoder = encoders[1]
    oldleft_encoder = encoders[0]

    oldangle_Encoder = 0.0
    oldangle_Gyro = 0.0

    #Ros stuff needs to be in the talker function as per:http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29

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

    orientation_input, angle_degrees, mag = point_orientation(0,0,3,3,angle) #dummy coordinates for now

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
        #print('gyro: %s' % angle_Gyro)

        dGyro = angle_Gyro - oldangle_Gyro
        #print('Delta gyro: %s' % dGyro)
        dEncoder = angle_Encoder - oldangle_Encoder
        #print('Delta Encoder: %s' % dEncoder)

        oldangle_Encoder = angle_Encoder
        #print('old encoder: %s' % oldangle_Encoder)
        oldangle_Gyro = angle_Gyro                                      
        #print('old gyro: %s' % oldangle_Gyro)
        #print('orientation_input = %s' % orientation_input)
        if abs(dGyro - dEncoder) < Threshold:
            angle += dGyro
        else:
            angle += dEncoder
        
        if ((angle - 50 <= orientation_input) and (orientation_input <= angle + 50)): #current orientation should just be angle of encoder or gyro
            run(mag)
            a_star.motors(0,0)  
        else:
            a_star.motors(-50,50)
        
        #print('angle_degrees = %s' % angle_degrees)
        #rate.sleep()
        #print('angle = %s' % angle)
        #print(sampleRate)

        #Add a time.sleep under this line if you think there should be one here
        
        sampleRate = timeit.default_timer() - start_time




if __name__ == '__main__':
    try:
        talker()
    except KeyboardInterrupt:
        a_star.motors(0,0)