#!/usr/bin/env python
import rospy, roslib, numpy
from romi_soccer.msg import Map
from romi_soccer.msg import Homography
# from map_calc import MapCalculator

# u1, u2, u3, u4 = None, None, None, None
# v1, v2, v3, v4 = None, None, None, None
# x1, x2, x3, x4 = None, None, None, None
# y1, y2, y3, y4 = None, None, None, None
# mat_h[] = None
# inv_mat[] = None
class ImageMapper():

    def __init__(self):
        # Initializes publishers
        rospy.loginfo('Initializing publishers...')
        self.pub_corner = rospy.Publisher('mapper/new_data/corners',Map,queue_size=10)
        self.pub_ball = rospy.Publisher('mapper/new_data/ball',Vector2,queue_size=10)
        self.pub_red_triangle = rospy.Publisher('mapper/new_data/red/triangle',Rover,queue_size=10)
        self.pub_red_square = rospy.Publisher('mapper/new_data/red/square',Rover,queue_size=10)
        self.pub_red_circle = rospy.Publisher('mapper/new_data/red/circle',Rover,queue_size=10)
        self.pub_blue_triangle = rospy.Publisher('mapper/new_data/blue/triangle',Rover,queue_size=10)
        self.pub_blue_square = rospy.Publisher('mapper/new_data/blue/square',Rover,queue_size=10)
        self.pub_blue_circle = rospy.Publisher('mapper/new_data/blue/circle',Rover,queue_size=10)
        rospy.loginfo('Done.')

        # Set the ROS rate to 10 Hz
        rate = rospy.Rate(10)
        # Initializes subscribers
        rospy.loginfo('Initializing subscribers...')
        rospy.Subscriber('mapper/raw_data/corners',Map, callback_calibrate)
        rospy.Subscriber('mapper/raw_data/ball',Vector2, callback_ball)
        rospy.Subscriber('mapper/raw_data/red/triangle',Rover, callback_red_triangle)
        rospy.Subscriber('mapper/raw_data/red/square',Rover, callback_red_square)
        rospy.Subscriber('mapper/raw_data/red/circle',Rover, callback_red_circle)
        rospy.Subscriber('mapper/raw_data/blue/triangle',Rover, callback_blue_triangle)
        rospy.Subscriber('mapper/raw_data/blue/square',Rover, callback_blue_square)
        rospy.Subscriber('mapper/raw_data/blue/circle',Rover, callback_blue_circle)
        rospy.loginfo('Done.')

        # Recalibrates the homography matrix after the subscriber initalizes
        # the pixel coordinates
        # while not rospy.is_shutdown():
        #     if u1 is not None:
        #         rospy.loginfo('Recalibrating homography matrix...')
        #         self.recalibrate()
        #         rospy.loginfo('Done.')
        #         rospy.loginfo('Publishing recalibrated homography matrix...')
        #         pub.publish(homography)
        #         rospy.loginfo('Done.')
        #     else: rospy.loginfo('Coordinates not initialized yet.')
        #     rate.sleep()

    def callback_calibrate(corner):
        rospy.loginfo('Received new coordinate data.')
        rospy.loginfo('Recalibrating new coordinate pairs (pixels <=> table)...')
        try:
            # Set origin to top left corner, offset the pixel coordinate by the
            # radius of the rover
            u1 = corner.TopL.x + 32
            v2 = corner.TopL.y + 32
            x1 = 0
            y1 = 0

            u2 = corner.BotL.x - 32
            v2 = corner.BotL.y + 32
            x2 = 5
            y2 = 0

            u3 = corner.TopR.x + 32
            v3 = corner.TopR.y - 32
            x3 = 5
            y3 = 10

            u4 = corner.BotR.x - 32
            v4 = corner.BotR.y - 32
            x4 = 0
            y4 = 10

        except rospy.ROSInterruptException:
            pass

        rospy.loginfo('Recalibrated pairs.')
        rospy.loginfo('Recalibrating homography matrix...')

        A = numpy.array([[x1, y1, 1, 0 , 0, 0, -u1*x1, -u1*y1],
                         [0, 0, 0, x1, y1, 1, -v1*x1, -v1*y1],
                         [x2, y2, 1, 0, 0, 0, -u2*x2, -u2*y2],
                         [0, 0, 0, x2, y2, 1, -v2*x2, -v2*y2],
                         [x3, y3, 1, 0, 0, 0, -u3*x3, -u3*y3],
                         [0, 0, 0, x3, y3, 1, -v3*x3, -v3*y3],
                         [x4, y4, 1, 0, 0, 0, -u4*x4, -u4*y4],
                         [0, 0, 0, x4, y4, 1, -v4*x4, -v4*y4]])

        b = numpy.array([[u1],
                        [v1],
                        [u2],
                        [v2],
                        [u3],
                        [v3],
                        [u4],
                        [v4]])

        x = numpy.linalg.solve(A,b)

        # # Store the homography 8x1 column vector into the message to publish
        # homography.h[0] = x[0,0]
        # homography.h[1] = x[1,0]
        # homography.h[2] = x[2,0]
        # homography.h[3] = x[3,0]
        # homography.h[4] = x[4,0]
        # homography.h[5] = x[5,0]
        # homography.h[6] = x[6,0]
        # homography.h[7] = x[7,0]

        # Make a square matrix from the homography 8x1 column vector
        mat_h = numpy.array([[x[0,0], x[1,0], x[2,0]],
                             [x[3,0], x[4,0], x[5,0]],
                             [x[6,0], x[7,0],     1]])

        # Take the inverse of the square homography matrix
        inv_mat = numpy.linalg.inv(mat_h)

        # # Store the inverse matrix into the message to publish
        # homography.q[0] = inv_mat[0,0]
        # homography.q[1] = inv_mat[0,1]
        # homography.q[2] = inv_mat[0,2]
        # homography.q[3] = inv_mat[1,0]
        # homography.q[4] = inv_mat[1,1]
        # homography.q[5] = inv_mat[1,2]
        # homography.q[6] = inv_mat[2,0]
        # homography.q[7] = inv_mat[2,1]
        # homography.q[8] = inv_mat[2,2]

    def map2pix(x,y):
