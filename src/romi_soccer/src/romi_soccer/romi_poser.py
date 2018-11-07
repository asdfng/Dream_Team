#!/usr/bin/env python
import rospy, roslib, numpy, tf2_ros, tf_conversions
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from romi_soccer.msg import Homography

class RomiPoser:
    def __init__(self):
        # Initializes a boolean to flag whether homography matrix has been received and stored yet
        self.grabbed = False
        # Initializes a boolean to flag whether this is the initial pose or not
        self.first = True
        # Initializes each element of the inverse homography matrix. Does this because when I grab the
        # homography matrix, I grab it one element at a time, since it makes the rest of the code
        # easier when I'm manipulating stuff with only a few elements at a time
        self.q11 = 0
        self.q12 = 0
        self.q13 = 0
        self.q21 = 0
        self.q22 = 0
        self.q23 = 0
        self.q31 = 0
        self.q32 = 0
        self.q33 = 0
        # Initializes a buffer for the tf2 listener
        self.tfBuffer = tf2_ros.Buffer()
        # Initializes a tf2 listener using the buffer
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        # Gets the name of the robot this transform is for (e.g. romi_white, romi_pink, romi_red)
        self.robot_name = rospy.get_param('robot_name')
        # Gets the name of the subject this transform is for (e.g. red_circle, blue_circle, etc.)
        subject = rospy.get_param('subject')
        # Initializes subscriber to homography topic
        rospy.Subscriber('/mapper/homography',Homography, self.matCallback)
        # Initializes subscriber to the JSON coordinate for the specific robot/subject specified by the parameters
        rospy.Subscriber('/%s/%s/raw_pose' % (subject,self.robot_name),PoseStamped, self.poseCallback)
        # Initializes a publisher for the new pose after being converted via homography
        self.pub = rospy.Publisher('/%s/%s/pose' % (subject,self.robot_name),PoseStamped, queue_size=10)
        rospy.spin()
        while not rospy.is_shutdown():
            self.tf_listener()
            self.tf_broadcaster()

    def matCallback(self,matrix):
        # Grabs the homography matrix from the homography topic.
        # Stores each element into class members.
        self.q11 = matrix.q11
        self.q12 = matrix.q12
        self.q13 = matrix.q13
        self.q21 = matrix.q21
        self.q22 = matrix.q22
        self.q23 = matrix.q23
        self.q31 = matrix.q31
        self.q32 = matrix.q32
        self.q33 = matrix.q33
        # Set flag to true now that we have our data.
        self.grabbed = True

    def poseCallback(self,data):
        u = data.pose.position.x
        v = data.pose.position.y
        if (self.grabbed):
            # Initializes an empty PoseStamped object
            new_pose = PoseStamped()
            new_pose.header.frame_id = 'odom_%s' % (self.robot_name)
            new_pose.header.stamp = rospy.Time.now()
            new_pose.pose.position.x = ((self.q11*u+self.q12*v+self.q13)/(self.q31*u+self.q32*v+self.q33))
            new_pose.pose.position.y = ((self.q21*u+self.q22*v+self.q23)/(self.q31*u+self.q32*v+self.q33))
            new_pose.pose.position.z = 0
            self.pub.publish(new_pose)
            if (self.first):
                rospy.set_param('/%s_first_pose_x' % self.robot_name, '%s' % new_pose.pose.position.x)
                rospy.set_param('/%s_first_pose_y' % self.robot_name, '%s' % new_pose.pose.position.y)
                self.first = False
            self.tf_listener(new_pose)

    def tf_listener(self,new_pose):
        # Set the rate to 10 Hz
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                # Look for the odom->base_link transform for the robot specified in the parameter server
                starscream = self.tfBuffer.lookup_transform('odom_%s' % (self.robot_name), 'base_link_%s' % (self.robot_name), rospy.Time())
                rospy.logdebug("Found the transform!")
                self.tf_broadcaster(new_pose,starscream)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                # Couldn't find the transform, try again
                rate.sleep()
                rospy.loginfo("Couldn't find transform from odom_%s to base_link_%s, trying again." %(self.robot_name, self.robot_name))
                continue


    def tf_broadcaster(self,new_pose,starscream):
        # Out of the loop, we have our transform now.
        # Initializes a tf2 broadcaster for our map->odom
        br = tf2_ros.TransformBroadcaster()
        # Initializes an empty TransformStamped object
        opt_prime = TransformStamped()
        # Sets the frame ID of the transform to the map frame
        opt_prime.header.frame_id = "map"
        # Stamps the transform with the current time
        opt_prime.header.stamp = rospy.Time.now()
        # Sets the child frame ID to odom, set in the poseCallback
        opt_prime.child_frame_id = new_pose.header.frame_id
        # Fill in the transform with the info we have.
        # The entire tf is map->odom->base_link, we're making the map->base_link by doing map->odom since
        # we can't do map->base_link directly because no child frame can have more than one parent frame.
        # So we find map->odom instead by finding what would have been map->base_link and then subtracting
        # the tf from odom->base_link.
        opt_prime.transform.translation.x = new_pose.pose.position.x - starscream.transform.translation.x
        opt_prime.transform.translation.y = new_pose.pose.position.y - starscream.transform.translation.y
        opt_prime.transform.translation.z = new_pose.pose.position.z - starscream.transform.translation.z
        # The JSON orientation sucks anyway so don't even bother. Just use the same rotation as odometry.
        opt_prime.transform.rotation = starscream.transform.rotation
        # Broadcast the transform.
        br.sendTransform(opt_prime)
