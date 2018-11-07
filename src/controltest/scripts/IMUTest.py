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
sampleRate = .01 #100Hz
#Change this to rospy.is_shutdown() to use this in ros 

i = 0
angle = 0.0
total = 0.0

while True:
    
    imu.read()

    #Find the offset and remove it
    while i<=10:
        
        imu.read()
        total += imu.g.z
        i += 1
    
    offsetGZ = total/10

    angle += imu.g.z*gyroSensitivity*sampleRate
    print("gyro orientation = %s" % angle)
    time.sleep(sampleRate)
    os.system('clear')

