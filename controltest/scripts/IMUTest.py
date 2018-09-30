#!/usr/bin/env python
#Code for getting the IMU data
#Written by: Nicholas Gregg

#This code should be run as a standalone Publisher, this way code is made
#very clear

import time
import os
from lms6 import LSM6

#Initiate the IMU and enable it
imu = LSM6()
imu.enable()

accelSensitivity = 0.061
accelRatio = .001 #Converting from milligrams to grams
gyroSensitivity = 0.035

#Change this to rospy.is_shutdown() to use this in ros 
while True:
    
    #read the IMU, print and wait
    imu.read()

    #Create a dictionary of converted values, use sensitivety data from the spec sheet
    imudata = {
        "gx": imu.g.x*gyroSensitivity,
        "gy": imu.g.y*gyroSensitivity,
        "gz": imu.g.z*gyroSensitivity,
        "ax": imu.a.x*accelSensitivity*accelRatio,  
        "ay": imu.a.y*accelSensitivity*accelRatio,
        "az": imu.a.z*accelSensitivity*accelRatio
    }

    #Gyro is in degrees per second while accel is in g
    print("G - x: %(gx)s y:%(gy)s z:%(gz)s \nA - x:%(ax)s y:%(ay)s z:%(az)s" % imudata)
    time.sleep(1)
    os.system('clear')

