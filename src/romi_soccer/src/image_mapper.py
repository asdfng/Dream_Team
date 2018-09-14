import rospy
import roslib
from romi_soccer.msg import Corner
from rospy.numpy_msg import numpy_msg
class ImageMapper():
    def __init__(self):
        init_subs()
        init_pubs()
# Initializes publishers
    def init_pubs():
        rospy.loginfo('Initializing publishers...')
        # Publisher of pixel coordinates taken from JSON
        pub_pix   = rospy.Publisher('mapper/pix_coordinates',Corner,queue_size=10)
        # Publisher of table coordinates converted from homography
        pub_table = rospy.Publisher('mapper/table_coordinates',CornerConverted,queue_size=10)
        rospy.loginfo('Done.')
        corner = Corner()
        while not rospy.is_shutdown():
            json_grabber()
            pub_pix.publish(corner)
            if rate:
                rospy.sleep(1/rate)
            else:
                rospy.sleep(1.0)

    def init_subs():
        rospy.loginfo('Initializing subscriber...')
        rospy.Subscriber('mapper/pix_coordinates',CornerConverted,callback)
        rospy.loginfo('Done.')

    def callback(data):
        rospy.loginfo('Received new coordinate data.')
        rospy.loginfo('Recalibrating...')
        recalibrate()

    def json_grabber():
        # Noah's code here

# Recalibrates homography matrix with new corner data
    def recalibrate():
        u1 = corner.pixel.TopL.x + 32
        v2 = corner.pixel.TopL.y + 32
        x1 = 0
        y1 = 0

        u2 = corner.pixel.BotL.x - 32
        v2 = corner.pixel.BotL.y + 32
        x2 = 5
        y2 = 0

        u3 = corner.pixel.TopR.x + 32
        v3 = corner.pixel.TopR.y - 32
        x3 = 5
        y3 = 10

        u4 = corner.pixel.BotR.x - 32
        v4 = corner.pixel.BotR.y - 32
        x4 = 0
        y4 = 10

        A = numpy.array([x1, y1, 1, 0 , 0, 0, -u1*x1, -u1*y1],
                        [0, 0, 0, x1, y1, 1, -v1*x1, -v1*y1],
                        [x2, y2, 1, 0, 0, 0, -u2*x2, -u2*y2],
                        [0, 0, 0, x2, y2, 1, -v2*x2, -v2*y2],
                        [x3, y3, 1, 0, 0, 0, -u3*x3, -u3*y3],
                        [0, 0, 0, x3, y3, 1, -v3*x3, -v3*y3],
                        [x4, y4, 1, 0, 0, 0, -u4*x4, -u4*y4],
                        [0, 0, 0, x4, y4, 1, -v4*x4, -v4*y4])

        b = numpy.array([u1],
                        [v1],
                        [u2],
                        [v2],
                        [u3],
                        [v3],
                        [u4],
                        [v4])
        x = numpy.linalg.solve(A,b)
        corner.homography.mat[0] = x[0,0]
        corner.homography.mat[1] = x[1,0]
        corner.homography.mat[2] = x[2,0]
        corner.homography.mat[3] = x[3,0]
        corner.homography.mat[4] = x[4,0]
        corner.homography.mat[5] = x[5,0]
        corner.homography.mat[6] = x[6,0]
        corner.homography.mat[7] = x[7,0]


    # # Convert top left coordinates
    #     A_TopL = numpy.array([x1, y1, 1, 0, 0, 0, -u1*x1, -u1*y1],
    #                          [0, 0, 0, x1, y1, 1, -v1*x1, -v1*y1])
    #     b_TopL = numpy.array([u1],
    #                          [v1])
    #     x_TopL = numpy.linalg.solve(A_TopL,b_TopL)
    # # Convert top right coordinates
    #     A_TopR = numpy.array([x2, y2, 1, 0, 0, 0, -u2*x2, -u2*y2],
    #                          [0, 0, 0, x2, y2, 1, -v2*x2, -v2*y2])
    #     b_TopR = numpy.array([u2],
    #                          [v2])
    #     x_TopR = numpy.linalg.solve(A_TopR,b_TopR)
    # # Convert bottom left coordinates
    #     A_BotL = numpy.array([x3, y3, 1, 0, 0, 0, -u3*x3, -u3*y3],
    #                          [0, 0, 0, x3, y3, 1, -v3*x3, -v3*y3])
    #     b_BotL = numpy.array([u3],
    #                          [v3])
    #     x_BotL = numpy.linalg.solve(A_BotL,b_BotL)
    # # Convert bottom right coordinates
    #     A_BotR = numpy.array([x4, y4, 1, 0, 0, 0, -u4*x4, -u4*y4],
    #                          [0, 0, 0, x4, y4, 1, -v4*x4, -v4*y4])
    #     b_BotR = numpy.array([u4],
    #                          [v4])
    #     x_BotR = numpy.linalg.solve(A_BotR,b_BotR)
