#!/usr/bin/env python
import rospy, keyboard
from controltest.msg import RoverDirections


rospy.init_node('Controller',anonymous=True)
pub = rospy.Publisher('/directions_rover', RoverDirections, queue_size=10)
rate = rospy.Rate(100)
directions = RoverDirections()

while not rospy.is_shutdown():
        
    #Determine if the keys have been pressed
        
    left = keyboard.is_pressed('a')
    right = keyboard.is_pressed('d')
    up = keyboard.is_pressed('w')
    down = keyboard.is_pressed('s')

    #set the message values to the keypress values

    directions.left = left
    directions.right = right
    directions.back = down
    directions.forward = up

    #Publish message
    pub.publish(directions)
    rate.sleep()

