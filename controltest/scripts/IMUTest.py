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
    
    #Find the offset and remove it
    while i<=10:
         total += imu.g.z
         i += 1
    
    offsetGZ = total/10

#Create a dictionary of converted values, use sensitivety data from the spec sheet
    imudata = {
        "gx": imu.g.x*gyroSensitivity,
        "gy": imu.g.y*gyroSensitivity,
        "gz": imu.g.z*gyroSensitivity - offsetGZ,
        "ax": imu.a.x*accelSensitivity*accelRatio,  
        "ay": imu.a.y*accelSensitivity*accelRatio,
        "az": imu.a.z*accelSensitivity*accelRatio
    }
    #read the IMU, print and wait
    imu.read()

    angle += imu.g.z*sampleRate

    #Gyro is in degrees per second while accel is in g
    #print("G - x: %(gx).2f y:%(gy).2f z:%(gz).2f \nA - x:%(ax).2f y:%(ay).2f z:%(az).2f" % imudata)
    print(angle)
    time.sleep(sampleRate)
    os.system('clear')
