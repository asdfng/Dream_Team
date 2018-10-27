#!/usr/bin/env python
import rospy, roslib, numpy, math, tf_conversions, tf2_ros, tf2_msgs, time, os
from geometry_msgs.msg import Pose2D, Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry

class OdomCalc:
    def __init__(self):
        # Boolean to flag whether it's starting from initial coordinates or not
        self.initial = True
        self.grabbed_vel = False
        self.grabbed_pose = False
        self.odom_quat = Quaternion()
        # Grabs the subject name (e.g. red_circle, blue_square, etc) from the parameter server in the launch file
        self.subject = rospy.get_param('subject')
        # Grabs the robot name (e.g. romi_pink, romi_red, romi_white) from the parameter server in the launch file.
        self.robot_name = rospy.get_param('robot_name')
        # Initializes publisher to odom topic
        self.pub_odom = rospy.Publisher('/%s/%s/romi_controller/odom' % (self.subject,self.robot_name),Odometry,queue_size=10)
        rospy.Subscriber('/%s/%s/pi_vel' % (self.subject, self.robot_name), Twist, self.velCallback)
        rospy.Subscriber('/%s/%s/pi_pose' % (self.subject, self.robot_name), Twist, self.poseCallback)
        self.new_x = 0.0
        self.new_y = 0.0
        self.vel = Twist()
        self.pose2D = Pose2D()
        self.rate = rospy.Rate(10)

    def velCallback(self, vel):
        self.vel = vel
        self.grabbed_vel = True

    def poseCallback(self, pose):
        self.pose2D = pose
        self.grabbed_pose = True

    def position_calculator(self,initial_x_coordinate, initial_y_coordinate, displacement_of_center, orientation_used):
        x_pose_change = displacement_of_center*math.cos(math.radians(orientation_used))
        y_pose_change = displacement_of_center*math.sin(math.radians(orientation_used))
        new_position_x = x_pose_change + initial_x_coordinate
        new_position_y = y_pose_change + initial_y_coordinate
        return new_position_x, new_position_y

    def broadcaster(self):
        # Noah, I don't know how you figured out where the new pose of the rover is,
        # but please put that stuff here. The transform needs to have a new calculated pose
        # of the robot on the table coordinates. I think you said you have that code around
        # here somewhere but I can't find it.

        # You can get (and should) rid of this stuff after you put your stuff in, it's just
        # there as a placeholder until you put your stuff in. Just make sure your stuff can give
        # coordinates of the calculated new position of the rover based on encoders. Also please
        # make it so when it starts up, it does NOT initialize to 0,0 but instead initializes to
        # the pose from the JSON coordinates. I already did that actually, just make sure that
        # the rest of your code works with it.

        # Initialize the starting coordinates like this:
        # If this is the first run through...
        if (self.initial):
            # Keep looking for the parameter until it's there
            while not rospy.has_param('/%s_first_pose_x' % self.robot_name):
                self.rate.sleep()
                continue
            # Store the parameters in x and y
            first_x = rospy.get_param('/%s_first_pose_x' % self.robot_name)
            first_y = rospy.get_param('/%s_first_pose_y' % self.robot_name)
            x = first_x
            y = first_y
            try:
                # Delete the parameters since we don't need them anymore
                rospy.delete_param('/%s_first_pose_x')
                rospy.delete_param('/%s_first_pose_y')
                # Mark that we've gone through the first time now
                self.initial = False
            except KeyError:
                rospy.loginfo('No initial coordinates to delete.')
        else:
            # Store the last value before we change them
            x = self.new_x
            y = self.new_y
        # Noah make sure it latches onto the old coordinates when it's not first booting up

        # Initializes empty TransformStamped object
        odom_trans = TransformStamped()
        # Stamps the transform with the current time
        odom_trans.header.stamp = self.current_time
        # Sets the frame ID of the transform to the odom frame
        odom_trans.header.frame_id = 'odom_%s' % self.robot_name
        # Sets the child frame ID to base_link
        odom_trans.child_frame_id = 'base_link_%s' % self.robot_name

        self.new_x, self.new_y = self.position_calculator(x, y, self.center_displacement, self.angle)
        odom_trans.transform.translation.x = self.new_x
        odom_trans.transform.translation.y = self.new_y
        odom_trans.transform.translation.z = 0
        odom_trans.transform.rotation = Quaternion(*self.odom_quat)

        br = tf2_ros.TransformBroadcaster()
        br.sendTransform(odom_trans)
        odom = Odometry()
        odom.header.stamp = self.current_time
        odom.header.frame_id = 'odom_%s' % self.robot_name

        self.current_time = rospy.Time.now();
        # dt = self.current_time-self.last_time
        # vx = self.center_displacement/dt
        # vth = (self.theta_new - self.theta_initial)/(self.theta_new_time - self.theta_initial_time)
        odom.pose.pose.position.x = self.new_x
        odom.pose.pose.position.y = self.new_y
        odom.pose.pose.position.z = 0

        th = self.pose2D.theta
        self.odom_quat = tf_conversions.transformations.quaternion_from_euler(0,0,th)
        odom.pose.pose.orientation = Quaternion(*self.odom_quat)

        odom.child_frame_id = 'base_link_%s' % self.robot_name
        odom.twist.twist.linear.x = self.vel.linear.x
        odom.twist.twist.linear.y = self.vel.linear.y
        odom.twist.twist.angular.z = self.vel.angular.z

        self.pub_odom.publish(odom)
        self.grabbed_pose = False
        self.grabbed_vel = False
