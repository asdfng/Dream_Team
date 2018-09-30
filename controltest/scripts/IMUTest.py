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

gyroXOffset = 2.00
gyroYOffset = -9.00
gyroZOffset = -5.00

accelSensitivity = 0.061
accelRatio = .001 #Converting from milligrams to grams
gyroSensitivity = 0.035

#Change this to rospy.is_shutdown() to use this in ros 
while True:
    
    #read the IMU, print and wait
    imu.read()

    #Create a dictionary of converted values, use sensitivety data from the spec sheet
    imudata = {
        "gx": imu.g.x*gyroSensitivity - gyroXOffset,
        "gy": imu.g.y*gyroSensitivity - gyroYOffset,
        "gz": imu.g.z*gyroSensitivity - gyroZOffset,
        "ax": imu.a.x*accelSensitivity*accelRatio,  
        "ay": imu.a.y*accelSensitivity*accelRatio,
        "az": imu.a.z*accelSensitivity*accelRatio
    }

    #Gyro is in degrees per second while accel is in g
    print("G - x: %(gx).2f y:%(gy).2f z:%(gz).2f \nA - x:%(ax).2f y:%(ay).2f z:%(az).2f" % imudata)
    time.sleep(.2)
    os.system('clear')

