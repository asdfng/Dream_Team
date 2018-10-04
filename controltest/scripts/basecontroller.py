#!/usr/bin/env python

#Code convert geometry/twist msgs to motor commands
#By: Nicholas Gregg

import rospy
from a_star import AStar
from geometry_msgs.msg import Twist


mTOft = 3.28084
a_star = AStar()

#converting the linear and angular message velocities to x and y
def callback(msg):
    #Only the angular z-axis and linear x- axis velocity are needed
    velXL = msg.linear.x * mTOft
    velZA = msg.angular.z * mTOft

    #Calculated max velocity to be 0.78508 ft/s
    #fitting -400 to 400 in the range of -0.78508 to 0.78508

    spLinear = int(((velXL + 0.78508)*(400 - (-400))/(.78508 - (-.78508))) - .78508)
    spAngular = int(((velZA + 0.78508)*(400 - (-400))/(.78508 - (-.78508))) - .78508)

    #the angular data is special it has to be converted to values
    #for the left and right motors, this will depend on the sign of the value

    spLeft = spLinear + spAngular
    spRight = spLinear - spAngular

    #Keep the number in an allowable range

    if spLeft > 400:
        spLeft = 400
    if spLeft < -400:
        spLeft = -400
    if spRight > 400:
        spRight = 400
    if spRight < -400:
        spRight = -400

    a_star.motors(spLeft,spRight)

#Setting up the subscriber node
def listener():
    rospy.init_node('MotorController', anonymous=True)
    rospy.Subscriber("/cmd_vel", Twist, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()