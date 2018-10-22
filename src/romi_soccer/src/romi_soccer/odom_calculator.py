#!/usr/bin/env python
import rospy, roslib, numpy, math, tf_conversions, tf2_ros, tf2_msgs, time, os
from a_star import AStar
from lms6 import LSM6
from geometry_msgs.msg import PoseStamped, Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry

class OdomCalc:
    def __init__(self):
        # Initialize the times
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        #Initialize all objects
        self.a_star = AStar()
        self.imu = LSM6()
        self.imu.enable()
        self.a_star.motors(-25,25)

        #Starting values and data for the imu
        self.accelSensitivity = 0.061
        self.accelRatio = 0.001 # Converting from milligrams to gram
        self.gyroSensitivity = 0.035
        self.sampleRate = .1 #100Hz
        self.odom_quat = Quaternion()
        # self.i = 0
        self.angle = 0.0
        self.angle_Gyro_unbounded = 0.0
        self.total = 0.0

        #Starting values for the encoders
        self.theta_initial = 0.0
        self.theta_initial_time = rospy.Time.now()
        self.theta_new_time = rospy.Time.now()
        self.theta_new = 0.0
        # Boolean to flag whether it's starting from initial coordinates or not
        self.initial = True
        self.center_displacement = 0.0

        # Grabs the subject name (e.g. red_circle, blue_square, etc) from the parameter server in the launch file
        self.subject = rospy.get_param('subject')
        # Grabs the robot name (e.g. romi_pink, romi_red, romi_white) from the parameter server in the launch file.
        self.robot_name = rospy.get_param('robot_name')
        # Initializes publisher to odom topic
        self.pub_odom = rospy.Publisher('/%s/%s/romi_controller/odom' % (self.subject,self.robot_name),Odometry,queue_size=10)
        self.talker()

    def displacement(self,right_encoder,left_encoder): #velocity: ft/s, position:
        pi = math.pi
        dist_between_wheels = 0.4791667
        self.theta_new_time = rospy.Time.now()
        #converts encoder counts to rotations
        right_wheel_rotations = right_encoder/float(1440)
        left_wheel_rotations = left_encoder/float(1440)

        #calculates displacement of right, left and center wheels
        right_displacement = right_wheel_rotations*float(2)*pi*.114829
        left_displacement = left_wheel_rotations*float(2)*pi*.114829
        self.center_displacement = (right_displacement + left_displacement)/float(2)
        #calculates the change of the angle by a turn
        alpha_left_turn_radians = (right_displacement - left_displacement)/dist_between_wheels

        #converts to degrees
        alpha_left_turn_degrees = alpha_left_turn_radians * float(180)/pi

        #appends initial theta to new theta
        theta_new_unbounded = self.theta_initial + alpha_left_turn_degrees
        self.theta_new = theta_new_unbounded % 360
        self.theta_initial = self.theta_new
        self.theta_initial_time = self.theta_new_time;

        return theta_new, self.center_displacement

    def get_odom_quat(self,dGyro,dEncoder,Threshold):
        if abs(dGyro - dEncoder) < Threshold:
            self.angle += dGyro
        else:
            self.angle += dEncoder
        self.odom_quat = tf_conversions.transformations.quaternion_from_euler(0,0,self.angle)

    def position_calculator(self,initial_x_coordinate, initial_y_coordinate, displacement_of_center, orientation_used):
        x_pose_change = displacement_of_center*math.cos(math.radians(orientation_used))
        y_pose_change = displacement_of_center*math.sin(math.radians(orientation_used))
        new_position_x = x_pose_change + initial_x_coordinate
        new_position_y = y_pose_change + initial_y_coordinate
        return new_position_x, new_position_y

    def talker(self):
        encoders = self.a_star.read_encoders()
        oldright_encoder = encoders[1]
        oldleft_encoder = encoders[0]
        oldangle_Encoder = 0.0
        oldangle_Gyro = 0.0
        self.last_time = rospy.Time.now()

        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            # start_time = timeit.default_timer()
            start_time = rospy.Time.now()
            Threshold = 0.125
            #Read the encoder and imu
            encoders = self.a_star.read_encoders()
            self.imu.read()
            #print(encoders[0], encoders[1])
            right_encoder = encoders[1]
            left_encoder = encoders[0]

            passRight = right_encoder - oldright_encoder
            passLeft = left_encoder - oldleft_encoder

            oldright_encoder = right_encoder
            oldleft_encoder = left_encoder

            self.angle_Encoder,self.center_displacement = self.displacement(passRight,passLeft)

            #print('Encoder: %s' % angle_Encoder)

            #Find the offset of the gyro and remove it
            i=10
            while i<=10:
                self.imu.read()
                self.total += self.imu.g.z
                i += 10

            offsetGZ = self.total/10

            self.angle_Gyro_unbounded += (self.imu.g.z*self.gyroSensitivity-offsetGZ)*self.sampleRate
            angle_Gyro = self.angle_Gyro_unbounded % 360
            rospy.loginfo('gyro: %s' % angle_Gyro)

            dGyro = angle_Gyro - oldangle_Gyro
            rospy.loginfo('Delta gyro: %s' % dGyro)
            dEncoder = angle_Encoder - oldangle_Encoder
            rospy.loginfo('Delta Encoder: %s' % dEncoder)

            oldangle_Encoder = angle_Encoder
            rospy.loginfo('old encoder: %s' % oldangle_Encoder)
            oldangle_Gyro = angle_Gyro
            rospy.loginfo('old gyro: %s' % oldangle_Gyro)
            self.get_odom_quat(dGyro,dEncoder,Threshold)
            angle_msg = Quaternion(*self.odom_quat)

            # rospy.loginfo(angle_msg)
            rospy.loginfo(self.sampleRate)
            rate.sleep() #Make sure this is equal to the output of the sample rate, DO NOT USE THE VARIABLE

            # self.sampleRate = timeit.default_timer() - start_time
            self.sampleRate = rospy.Time.now() - start_time
            self.broadcaster()

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
                rate.sleep()
                continue
            # Store the parameters in x and y
            x = rospy.get_param('/%s_first_pose_x' % self.robot_name)
            y = rospy.get_param('/%s_first_pose_y' % self.robot_name)
            try:
                # Delete the parameters since we don't need them anymore
                rospy.delete_param('/%s_first_pose_x')
                rospy.delete_param('/%s_first_pose_y')
                # Mark that we've gone through the first time now
                self.initial = False
            except KeyError:
                rospy.loginfo('No initial coordinates to delete.')
        # Noah make sure it latches onto the old coordinates when it's not first booting up

        # Initializes empty TransformStamped object
        odom_trans = TransformStamped()
        # Stamps the transform with the current time
        odom_trans.header.stamp = self.current_time
        # Sets the frame ID of the transform to the odom frame
        odom_trans.header.frame_id = 'odom_%s' % self.robot_name
        # Sets the child frame ID to base_link
        odom_trans.child_frame_id = 'base_link_%s' % self.robot_name

        # Noah, please put math here to find the calculated new pose of the rover!!
        new_x, new_y = self.position_calculator(x, y, self.center_displacement, self.angle)
        odom_trans.transform.translation.x = new_x
        odom_trans.transform.translation.y = new_y
        odom_trans.transform.translation.z = 0
        odom_trans.transform.rotation = Quaternion(*self.odom_quat)

        br = tf2_ros.TransformBroadcaster()
        br.sendTransform(odom_trans)
        odom = Odometry()
        odom.header.stamp = self.current_time
        odom.header.frame_id = 'odom_%s' % self.robot_name

        self.current_time = rospy.Time.now();
        dt = self.current_time-self.last_time
        vx = self.center_displacement/dt
        vth = (self.theta_new - self.theta_initial)/(self.theta_new_time - self.theta_initial_time)
        odom.pose.pose.position.x = new_x
        odom.pose.pose.position.y = new_y
        odom.pose.pose.position.z = 0
        # odom_quat = tf_conversions.transformations.quaternion_from_euler(0,0,th)
        odom.pose.pose.orientation = Quaternion(*self.odom_quat)

        odom.child_frame_id = 'base_link_%s' % self.robot_name
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = vth

        self.pub_odom.publish(odom)
        self.last_time = self.current_time
        self.grabbed_pose = False
        self.grabbed_vel = False
