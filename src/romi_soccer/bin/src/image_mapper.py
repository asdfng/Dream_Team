#!/usr/bin/env python
import rospy, roslib, numpy
from romi_soccer.msg import Map, Vector2, Rover

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
        self.init_subs()

    def init_subs(self):
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
        rospy.spin()

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

        # Make a square matrix from the homography 8x1 column vector
        self.mat_h = numpy.array([[x[0,0], x[1,0], x[2,0]],
                             [x[3,0], x[4,0], x[5,0]],
                             [x[6,0], x[7,0],     1]])

        # Take the inverse of the square homography matrix
        self.inv_mat = numpy.linalg.inv(mat_h)
        table = Map()
        table.TopL.x = self.mapx2tab(u1,v1)
        table.TopL.y = self.mapy2tab(u1,v1)

        table.TopR.x = self.mapx2tab(u1,v1)
        table.TopR.y = self.mapy2tab(u1,v1)

        table.BotL.x = self.mapx2tab(u1,v1)
        table.BotL.y = self.mapy2tab(u1,v1)

        table.BotR.x = self.mapx2tab(u1,v1)
        table.BotR.y = self.mapy2tab(u1,v1)

        self.pub_corner.publish(table)


    def callback_ball(ball):
        ball_new = Vector2()
        ball_new.x = self.mapx2tab(ball.x, ball.y)
        ball_new.y = self.mapy2tab(ball.x, ball.y)
        self.pub_ball.publish(ball_new)

    def callback_red_triangle(rover_old):
        rover_new = Rover()
        rover_new.x = self.mapx2tab(rover_old.x, rover_old.y)
        rover_new.y = self.mapy2tab(rover_old.x, rover_old.y)
        self.pub_red_triangle.publish(rover_new)

    def callback_red_square(rover_old):
        rover_new = Rover()
        rover_new.x = self.mapx2tab(rover_old.x, rover_old.y)
        rover_new.y = self.mapy2tab(rover_old.x, rover_old.y)
        self.pub_red_square.publish(rover_new)

    def callback_blue_triangle(rover_old):
        rover_new = Rover()
        rover_new.x = self.mapx2tab(rover_old.x, rover_old.y)
        rover_new.y = self.mapy2tab(rover_old.x, rover_old.y)
        self.pub_blue_triangle.publish(rover_new)

    def callback_blue_square(rover_old):
        rover_new = Rover()
        rover_new.x = self.mapx2tab(rover_old.x, rover_old.y)
        rover_new.y = self.mapy2tab(rover_old.x, rover_old.y)
        self.pub_blue_square.publish(rover_new)

    def callback_blue_circle(rover_old):
        rover_new = Rover()
        rover_new.x = self.mapx2tab(rover_old.x, rover_old.y)
        rover_new.y = self.mapy2tab(rover_old.x, rover_old.y)
        self.pub_blue_circle.publish(rover_new)


    def mapx2tab(u,v):
        q11 = self.inv_mat[0,0]
        q12 = self.inv_mat[0,1]
        q13 = self.inv_mat[0,2]
        q21 = self.inv_mat[1,0]
        q22 = self.inv_mat[1,1]
        q23 = self.inv_mat[1,2]
        q31 = self.inv_mat[2,0]
        q32 = self.inv_mat[2,1]
        q33 = self.inv_mat[2,2]
        return ((q11*u+q12*v+q13)/(q31*u+q32*v+q33))

    def mapy2tab(u,v):
        q11 = self.inv_mat[0,0]
        q12 = self.inv_mat[0,1]
        q13 = self.inv_mat[0,2]
        q21 = self.inv_mat[1,0]
        q22 = self.inv_mat[1,1]
        q23 = self.inv_mat[1,2]
        q31 = self.inv_mat[2,0]
        q32 = self.inv_mat[2,1]
        q33 = self.inv_mat[2,2]
        return ((q21*u+q22*v+q23)/(q31*u+q32*v+q33))

    def mapy2pix(x,y):
        h11 = self.mat_h[0,0]
        h12 = self.mat_h[0,1]
        h13 = self.mat_h[0,2]
        h21 = self.mat_h[1,0]
        h22 = self.mat_h[1,1]
        h23 = self.mat_h[1,2]
        h31 = self.mat_h[2,0]
        h32 = self.mat_h[2,1]
        return ((h11*x+h12*y+h13)/(h31*x+h32*y+1))

    def mapx2pix(x,y):
        h11 = self.mat_h[0,0]
        h12 = self.mat_h[0,1]
        h13 = self.mat_h[0,2]
        h21 = self.mat_h[1,0]
        h22 = self.mat_h[1,1]
        h23 = self.mat_h[1,2]
        h31 = self.mat_h[2,0]
        h32 = self.mat_h[2,1]
        return ((h21*x+h22*y+h23)/(h31*x+h32*y+1))
