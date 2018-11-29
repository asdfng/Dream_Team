#!/usr/bin/env python
import rospy, roslib, numpy
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import PoseStamped, Point32, Vector3Stamped
from romi_soccer.msg import Homography

class RomiFinderSimple:
    def __init__(self,cloud):
        # Initializes a boolean to flag whether the homography matrix has been grabbed yet
        self.cloud = cloud
        self.grabbed = False
        self.q11 = 0
        self.q12 = 0
        self.q13 = 0
        self.q21 = 0
        self.q22 = 0
        self.q23 = 0
        self.q31 = 0
        self.q32 = 0
        self.q33 = 0
        self.subject = rospy.get_param('subject')
        self.subject_robot = rospy.get_param('robot_name')
        romi_circle = rospy.get_param('romi_circle')
        romi_square = rospy.get_param('romi_square')
        romi_triangle = rospy.get_param('romi_triangle')
        if self.subject.startswith("red"):
            team="red"
            other="blue"
        elif self.subject.startswith("blue"):
            team="blue"
            other="red"
        if self.subject.endswith("circle"):
            rospy.Subscriber('/%s_square/raw_pose' % other,PoseStamped,self.callback1)
            rospy.Subscriber('/%s_circle/raw_pose' % other,PoseStamped,self.callback2)
            rospy.Subscriber('/%s_triangle/raw_pose' % other,PoseStamped,self.callback3)
            rospy.Subscriber('/%s_square/%s/raw_pose' % (team,romi_square),PoseStamped,self.callback4)
            rospy.Subscriber('/%s_triangle/%s/raw_pose' % (team,romi_triangle),PoseStamped,self.callback5)
        elif self.subject.endswith("square"):
            rospy.Subscriber('/%s_square/raw_pose' % other,PoseStamped,self.callback1)
            rospy.Subscriber('/%s_circle/raw_pose' % other,PoseStamped,self.callback2)
            rospy.Subscriber('/%s_triangle/raw_pose' % other,PoseStamped,self.callback3)
            rospy.Subscriber('/%s_circle/%s/raw_pose' %  (team,romi_circle),PoseStamped,self.callback4)
            rospy.Subscriber('/%s_triangle/%s/raw_pose' % (team,romi_triangle),PoseStamped,self.callback5)
        elif self.subject.endswith("triangle"):
            rospy.Subscriber('/%s_square/raw_pose' % other,PoseStamped,self.callback1)
            rospy.Subscriber('/%s_circle/raw_pose' % other,PoseStamped,self.callback2)
            rospy.Subscriber('/%s_triangle/raw_pose' % other,PoseStamped,self.callback3)
            rospy.Subscriber('/%s_circle/%s/raw_pose' % (team,romi_circle),PoseStamped,self.callback4)
            rospy.Subscriber('/%s_square/%s/raw_pose' % (team,romi_square),PoseStamped,self.callback5)
        self.pub = rospy.Publisher('/%s/%s/cloudy_meatballs' % (self.subject,self.subject_robot),PointCloud,queue_size=10)
        rospy.Subscriber('/mapper/homography',Homography, self.matCallback)
        rospy.Subscriber('/mapper/ball/raw_pose',Vector3Stamped,self.callback0)
        rospy.spin()

    def matCallback(self,matrix):
        self.q11 = matrix.q11
        self.q12 = matrix.q12
        self.q13 = matrix.q13
        self.q21 = matrix.q21
        self.q22 = matrix.q22
        self.q23 = matrix.q23
        self.q31 = matrix.q31
        self.q32 = matrix.q32
        self.q33 = matrix.q33
        self.grabbed = True

    def callback0(self,ball):
        if (self.grabbed):
            u = ball.vector.x
            v = ball.vector.y
            x = ((self.q11*u+self.q12*v+self.q13)/(self.q31*u+self.q32*v+self.q33))
            y = ((self.q21*u+self.q22*v+self.q23)/(self.q31*u+self.q32*v+self.q33))
            self.cloud.points[0] = Point32(x,y,0)
            self.pub.publish(self.cloud)
        else: rospy.loginfo('Waiting for homography calibration...')

    def callback1(self,data):
        if (self.grabbed):
            u = data.pose.position.x
            v = data.pose.position.y
            x = ((self.q11*u+self.q12*v+self.q13)/(self.q31*u+self.q32*v+self.q33))
            y = ((self.q21*u+self.q22*v+self.q23)/(self.q31*u+self.q32*v+self.q33))
            self.cloud.points[1] = Point32(x,y,0)
            self.pub.publish(self.cloud)
        else: rospy.loginfo('Waiting for homography calibration...')

    def callback2(self,data):
        if (self.grabbed):
            u = data.pose.position.x
            v = data.pose.position.y
            x = ((self.q11*u+self.q12*v+self.q13)/(self.q31*u+self.q32*v+self.q33))
            y = ((self.q21*u+self.q22*v+self.q23)/(self.q31*u+self.q32*v+self.q33))
            self.cloud.points[2] = Point32(x,y,0)
            self.pub.publish(self.cloud)
        else: rospy.loginfo('Waiting for homography calibration...')


    def callback3(self,data):
        if (self.grabbed):
            u = data.pose.position.x
            v = data.pose.position.y
            x = ((self.q11*u+self.q12*v+self.q13)/(self.q31*u+self.q32*v+self.q33))
            y = ((self.q21*u+self.q22*v+self.q23)/(self.q31*u+self.q32*v+self.q33))
            self.cloud.points[3] = Point32(x,y,0)
            self.pub.publish(self.cloud)
        else: rospy.loginfo('Waiting for homography calibration...')

    def callback4(self,data):
        if (self.grabbed):
            u = data.pose.position.x
            v = data.pose.position.y
            x = ((self.q11*u+self.q12*v+self.q13)/(self.q31*u+self.q32*v+self.q33))
            y = ((self.q21*u+self.q22*v+self.q23)/(self.q31*u+self.q32*v+self.q33))
            self.cloud.points[4] = Point32(x,y,0)
            self.pub.publish(self.cloud)
        else: rospy.loginfo('Waiting for homography calibration...')

    def callback5(self,data):
        if (self.grabbed):
            u = data.pose.position.x
            v = data.pose.position.y
            x = ((self.q11*u+self.q12*v+self.q13)/(self.q31*u+self.q32*v+self.q33))
            y = ((self.q21*u+self.q22*v+self.q23)/(self.q31*u+self.q32*v+self.q33))
            self.cloud.points[5] = Point32(x,y,0)
            self.pub.publish(self.cloud)
        else: rospy.loginfo('Waiting for homography calibration...')
