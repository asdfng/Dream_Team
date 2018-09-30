#!/usr/bin/env python
import rospy, roslib, json, time, tf2_ros
import urllib2
import geometry_msgs.msg

def world_to_map(homography):
    self.matrix = homography
    br = tf2_ros.TransformBroadcaster()
    opt_prime = geometry_msgs.msg.TransformStamped()
    opt_prime.header.stamp = rospy.Time.now()
    opt_prime.header.frame_id = 'world'
    opt_prime.child_frame_id = 'map'

    # q11 = self.matrix.q[0]
    # q12 = self.matrix.q[1]
    # q13 = self.matrix.q[2]
    # q21 = self.matrix.q[3]
    # q22 = self.matrix.q[4]
    # q23 = self.matrix.q[5]
    # q31 = self.matrix.q[6]
    # q32 = self.matrix.q[7]
    # q33 = self.matrix.q[8]
    # u = rover.pose.position.x
    # v = rover.pose.position.y
    # opt_prime.transform.translation.x = ((q11*u+q12*v+q13)/(q31*u+q32*v+q33))
    # opt_prime.transform.translation.y = ((q21*u+q22*v+q23)/(q31*u+q32*v+q33))
    opt_prime.transform.translation.x =
    opt_prime.transform.rotation.x = 0
    opt_prime.transform.rotation.y = 0
    opt_prime.transform.rotation.z = 0
    opt_prime.transform.rotation.w = 1
    br.sendTransform(opt_prime)

if __name__ == '__main__':
    rospy.init_node('tf_world_map_broadcaster')
    self.matrix = Homography()
    rospy.Subscriber('mapper/raw_data/corners', Map, world_to_map)
    rospy.spin()
