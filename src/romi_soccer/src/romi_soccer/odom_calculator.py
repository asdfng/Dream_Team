#!/usr/bin/env python
import rospy, roslib, numpy, math, tf_conversions, tf2_ros
from geometry_msgs.msg import PoseStamped, Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry

class OdomCalc:
    def __init__(self):
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        self.grabbed_vel = False
        self.grabbed_pose = False
        self.vel = Twist()
        self.pose = PoseStamped()
        self.subject = rospy.get_param('~subject')
        self.robot_name = rospy.get_param('~robot_name')
        self.pub = rospy.Publisher('%s/odom' % self.subject,Odometry,queue_size=10)
        rospy.Subscriber('%s/pose' % self.subject,PoseStamped,self.poseCallback)
        rospy.Subscriber('/%s/romi_controller/cmd_vel' % self.robot_name,Twist,self.callback)
        rospy.spin()

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
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        z = self.pose.pose.position.z
        th = self.pose.pose.orientation.z

        vx = self.vel.linear.x
        vy = self.vel.linear.y
        vth = self.vel.angular.z
        dt = (self.current_time - self.last_time).toSec()
        delta_x = (vx * math.cos(th) - vy * math.sin(th)) * dt
        delta_y = (vx * math.sin(th) + vy * math.cos(th)) * dt
        delta_th = vth * dt

        x = x + delta_x
        y = y + delta_y
        th = th + delta_th

        odom_trans = TransformStamped()
        odom_trans.header.stamp = self.current_time
        odom_trans.header.frame_id = 'odom_%s' % self.subject
        odom_trans.child_frame_id = 'base_link_%s' % self.subject

        odom_trans.transform.translation.x = x
        odom_trans.transform.translation.y = y
        odom_trans.transform.translation.z = 0
        odom_trans.transform.rotation = tf_conversions.transformations.quaternion_from_euler(0,0,th)

        br = tf2_ros.TransformBroadcaster()
        br.sendTransform(odom_trans)

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom_%s' % self.subject

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0
        odom.pose.pose.orientation = tf_conversions.transformations.quaternion_from_euler(0,0,th)

        odom.child_frame_id = 'base_link_%s' % self.subject
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vth

        self.pub.publish(odom)
        self.last_time = self.current_time
        self.grabbed_pose = False
        self.grabbed_vel = False
