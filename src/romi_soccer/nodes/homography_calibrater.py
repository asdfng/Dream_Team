#!/usr/bin/env python
import rospy, roslib, numpy
from romi_soccer.msg import Homography

def callback(corner):
    # Initialize publisher
    rospy.loginfo('Received new coordinate data.')
    rospy.loginfo('Recalibrating new coordinate pairs (pixels <=> table)...')
    # Set origin to top left corner, offset the pixel coordinate by the
    # radius of the rover
    u1 = corner.BotL.x
    v1 = corner.BotL.y
    # u1 = corner.BotL.x + 32
    # v1 = corner.BotL.y + 32
    x1 = 0
    y1 = 0

    rospy.loginfo('Received table coordinate (%s,%s) as pixel coordinates (%s,%s).',x1,y1,corner.BotL.x, corner.BotL.y)
    # rospy.loginfo('Offset origin by 32 pixels to adjust for radius of rover.')
    # rospy.loginfo('Set pixel coordinates to (%s,%s).',u1,v1)

    u2 = corner.BotR.x
    v2 = corner.BotR.y
    # u2 = corner.BotR.x - 32
    # v2 = corner.BotR.y + 32
    x2 = 3.9
    y2 = 0

    rospy.loginfo('Received table coordinate (%s,%s) as pixel coordinates (%s,%s).',x2,y2,corner.BotR.x, corner.BotR.y)
    # rospy.loginfo('Offset origin by 32 pixels to adjust for radius of rover.')
    # rospy.loginfo('Set pixel coordinates to (%s,%s).',u2,v2)

    u3 = corner.TopL.x
    v3 = corner.TopL.y
    # u3 = corner.TopL.x + 32
    # v3 = corner.TopL.y - 32
    x3 = 0
    y3 = 8

    rospy.loginfo('Received table coordinate (%s,%s) as pixel coordinates (%s,%s).',x3,y3,corner.TopL.x, corner.TopL.y)
    # rospy.loginfo('Offset origin by 32 pixels to adjust for radius of rover.')
    # rospy.loginfo('Set pixel coordinates to (%s,%s).',u3,v3)

    u4 = corner.TopR.x
    v4 = corner.TopR.y
    # u4 = corner.TopR.x - 32
    # v4 = corner.TopR.y - 32
    x4 = 3.9
    y4 = 8

    rospy.loginfo('Received table coordinate (%s,%s) as pixel coordinates (%s,%s).',x4,y4,corner.TopR.x, corner.TopR.y)
    # rospy.loginfo('Offset origin by 32 pixels to adjust for radius of rover.')
    # rospy.loginfo('Set pixel coordinates to (%s,%s).',u4,v4)

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
    # Make a square matrix from the homography 8x1 column vector
    mat_h = numpy.array([[x[0,0], x[1,0], x[2,0]],
                         [x[3,0], x[4,0], x[5,0]],
                         [x[6,0], x[7,0],     1]])

    # Take the inverse of the square homography matrix
    mat_q = numpy.linalg.inv(mat_h)
    homography = Homography()
    homography.h11 = x[0,0]
    homography.h12 = x[1,0]
    homography.h13 = x[2,0]
    homography.h21 = x[3,0]
    homography.h22 = x[4,0]
    homography.h23 = x[5,0]
    homography.h31 = x[6,0]
    homography.h32 = x[7,0]
    homography.h33 = 1

    homography.q11 = mat_q[0,0]
    homography.q12 = mat_q[0,1]
    homography.q13 = mat_q[0,2]
    homography.q21 = mat_q[1,0]
    homography.q22 = mat_q[1,1]
    homography.q23 = mat_q[1,2]
    homography.q31 = mat_q[2,0]
    homography.q32 = mat_q[2,1]
    homography.q33 = mat_q[2,2]
    pub.publish(homography)

if __name__ == '__main__':
    rospy.init_node('tf_calibrater')
    pub = rospy.Publisher('mapper/homography',Homography,queue_size=10)
    rospy.Subscriber('mapper/raw_data/corners',Map, callback)
    rospy.spin()
