#!/usr/bin/env python
import rospy, roslib, numpy
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32

def cloud_maker():
    cloud = PointCloud()
    cloud.header.stamp = rospy.Time.now()
    cloud.points = [Point32()]*6
    return cloud
