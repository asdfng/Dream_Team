#!/usr/bin/env python
import rospy, roslib, numpy, math
from geometry_msgs.msg import PoseStamped,Twist
from nav_msgs.msg import Odometry

class OdomCalc:
    def __init__(self):
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        self.pub = rospy.Publisher('odom',Odometry,queue_size=10)
        self.grabbed_vel = False
        self.grabbed_pose = False
        self.vel = Twist()
        self.pose = PoseStamped()
        rospy.Subscriber('pose',PoseStamped,self.poseCallback)
        rospy.Subscriber('cmd_vel',Twist,self.callback)

    def callback(self,data):
        self.vel = data
        self.grabbed_pose = True
        if (self.grabbed_vel):
            self.broadcaster()

    def poseCallback(self,data):
        self.pose = data
        self.grabbed_vel = True
        if (self.grabbed_pose):
            self.broadcaster()

    def broadcaster(self):
        self.current_time = rospy.Time.now()
        vx = self.vel.linear.x
        vy = self.vel.linear.y
        vth = self.vel.angular.z
        th = self.pose.pose.orientation.z
        dt = (self.current_time - self.last_time).toSec()
        delta_x = (vx * math.cos(th) - vy * math.sin(th)) * dt
        delta_y = (vx * math.sin(th) + vy * math.cos(th)) * dt
        delta_th = vth * dt
