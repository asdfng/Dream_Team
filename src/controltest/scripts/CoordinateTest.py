#!/usr/bin/env python
#import rospy
import time, json, urllib2
import timeit 
import os
import math
from lms6 import LSM6
from a_star import AStar

while True:
    response = urllib2.urlopen('http://192.168.137.1:8001/FieldData/GetData')
    source = response.read()
    data = json.loads(source.decode())
    red_square_x = data['Red Team Data']['Square']['Object Center']['X']
    red_square_y = data['Red Team Data']['Square']['Object Center']['Y']
    ball_x = data['Ball']['Object Center']['X']
    ball_y = data['Ball']['Object Center']['Y']
    mRSX = float(red_square_x - 12)*float(8/float(394-12))
    mRSY = float(red_square_y - 31)*float(8/float(221-31))
    mBX = float(ball_x - 12)*float(8)
    mBY = float(ball_y - 31)*float(8)
    print(red_square_x)
    print(type(red_square_x))
    print(red_square_y)
    print(ball_x)
    print(ball_y)
    print(mRSX)
    print(mRSY)
    print(mBX)
    print(mBY)
    time.sleep(1)