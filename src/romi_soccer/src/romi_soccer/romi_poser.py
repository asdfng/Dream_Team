#!/usr/bin/env python
import rospy, roslib, numpy, tf2_ros, tf_conversions
from geometry_msgs.msg import PoseStamped, TransformStamped
from romi_soccer.msg import Homography

class RomiPoser:
    def __init__(self):
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
        self.robot_name = rospy.get_param('robot_name')
        rospy.Subscriber('/mapper/homography',Homography, self.matCallback)
        rospy.Subscriber('/%s/raw_pose' % self.robot_name,PoseStamped, self.callback)
        self.pub = rospy.Publisher('/%s/romi_controller/pose' % self.robot_name,PoseStamped, queue_size=10)
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

    def callback(self,data):
        u = data.pose.position.x
        v = data.pose.position.y
        if (self.grabbed):
            new_pose = PoseStamped()
            new_pose.header.frame_id = 'odom_%s' % self.robot_name
            new_pose.header.stamp = rospy.Time.now()
            new_pose.pose.position.x = ((self.q11*u+self.q12*v+self.q13)/(self.q31*u+self.q32*v+self.q33))
            new_pose.pose.position.y = ((self.q21*u+self.q22*v+self.q23)/(self.q31*u+self.q32*v+self.q33))
            new_pose.pose.position.z = 0
            # Save this for when IMU data is parsed
            # th = angle from IMU
            # angle = tf_conversions.transformations.quaternion_from_euler(0,0,th)
            # new_pose.pose.orientation.x = angle[0]
            # new_pose.pose.orientation.y = angle[1]
            # new_pose.pose.orientation.z = angle[2]
            # new_pose.pose.orientation.w = angle[3]
            self.tf_broadcaster(new_pose)
            self.pub.publish(new_pose)

    def tf_broadcaster(self,new_pose):
        br = tf2_ros.TransformBroadcaster()
        opt_prime = TransformStamped()
        opt_prime.header.frame_id('map')
        opt_prime.child_frame_id(new_pose.header.frame_id)
        opt_prime.transform.translation.x = new_pose.pose.position.x
        opt_prime.transform.translation.y = new_pose.pose.position.y
        opt_prime.transform.translation.z = new_pose.pose.position.z
        # opt_prime.transform.rotation.x = new_pose.pose.orientation.x
        # opt_prime.transform.rotation.y = new_pose.pose.orientation.y
        # opt_prime.transform.rotation.z = new_pose.pose.orientation.z
        # opt_prime.transform.rotation.w = new_pose.pose.orientation.w
        br.sendTransform(opt_prime)
