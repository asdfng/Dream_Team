#!/usr/bin/env python
import rospy, roslib, numpy
from romi_soccer.msg import Map
from romi_soccer.msg import Homography

u1, v1, u2, v2, u3, v3, u4, v4 = 1, 2, 3, 4, 5, 6, 7, 8
x1, y1, x2, y2, x3, y3, x4, y4 = 1, 2, 3, 4, 5, 6, 7, 8
homography = Homography()

# from rospy.numpy_msg import numpy_msg as numpy
class ImageMapper():

    def __init__(self):
        # Initializes publishers
        rospy.loginfo('Initializing publishers...')
        pub = rospy.Publisher('mapper/homography',Homography,queue_size=10)
        pub.publish(homography)
        # pub_corner = rospy.Publisher('mapper/corner',Map,queue_size=10)
        rospy.loginfo('Done.')

        rate = rospy.Rate(10)
        rospy.loginfo('Initializing subscribers...')
        rospy.Subscriber('mapper/raw_data/corners',Map, self.recalibrate)
        rospy.loginfo('Done.')
        while not rospy.is_shutdown():
            self.recalibrate()
            rate.sleep()

    def callback(corner):
        rospy.loginfo('Received new coordinate data.')
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
        rospy.loginfo('Recalibrating...')


# Recalibrates homography matrix with new corner data
    def recalibrate(self):
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
        homography.h[0] = x[0,0]
        homography.h[1] = x[1,0]
        homography.h[2] = x[2,0]
        homography.h[3] = x[3,0]
        homography.h[4] = x[4,0]
        homography.h[5] = x[5,0]
        homography.h[6] = x[6,0]
        homography.h[7] = x[7,0]

        inv_mat = numpy.reciprocal(x)
        homography.q[0] = inv_mat[0,0]
        homography.q[1] = inv_mat[1,0]
        homography.q[2] = inv_mat[2,0]
        homography.q[3] = inv_mat[3,0]
        homography.q[4] = inv_mat[4,0]
        homography.q[5] = inv_mat[5,0]
        homography.q[6] = inv_mat[6,0]
        homography.q[7] = inv_mat[7,0]

        
