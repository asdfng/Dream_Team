#! /usr/bin/env python
import rospy
import roslib
from romi_soccer.msg import Map, Vector2, Rover
from rospy.numpy_msg import numpy_msg
import json, urllib, time

class JSONGrabber:
    def __init__(self):
        # Initializing publishers
        rospy.loginfo('Initializing publisher...')
        self.pub_corner = rospy.Publisher('mapper/raw_data/corners',Map,queue_size=10)
        self.pub_ball = rospy.Publisher('mapper/raw_data/ball',Vector2,queue_size=10)
        self.pub_red_triangle = rospy.Publisher('mapper/raw_data/red/triangle',Rover,queue_size=10)
        self.pub_red_square = rospy.Publisher('mapper/raw_data/red/square',Rover,queue_size=10)
        self.pub_red_circle = rospy.Publisher('mapper/raw_data/red/circle',Rover,queue_size=10)
        self.pub_blue_triangle = rospy.Publisher('mapper/raw_data/blue/triangle',Rover,queue_size=10)
        self.pub_blue_square = rospy.Publisher('mapper/raw_data/blue/square',Rover,queue_size=10)
        self.pub_blue_circle = rospy.Publisher('mapper/raw_data/blue/circle',Rover,queue_size=10)
        rospy.loginfo('Done.')

        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            with urllib.request.urlopen('http://172.16.0.1:8001/FieldData/GetData') as response:
                source = response.read()
                data = json.loads(source.decode())
                #Red Team data parsing
                self.Red = data['Red Team Data']
                #Blue Team data parsing
                self.Blue = data['Blue Team Data']

                #Red Circle
                self.rCircle = Red['Circle']
                self.rcCenterPoint = rCircle['Object Center']
                #Bounding Box
                self.rcBoundinBox = rCircle['Bounding Box']
                json_red_circle_grabber()

                #Red Square
                self.rSquare = Red['Square']
                self.rsCenterPoint = rSquare['Object Center']
                #Bounding Box
                self.rsBoundinBox = rSquare['Bounding Box']
                json_red_square_grabber()

                #Red Triangle
                self.rTriangle = Red['Triangle']
                self.rtCenterPoint = rTriangle['Object Center']
                #Bounding Btox
                self.rtBoundinBox = rTriangle['Bounding Box']
                json_red_triangle_grabber()

                #Blue Circle
                self.bCircle = Blue['Circle']
                self.bcCenterPoint = bCircle['Object Center']
                #Bounding Box
                self.bcBoundinBox = bCircle['Bounding Box']
                json_blue_circle_grabber()

                #Blue Square
                self.bSquare = Blue['Square']
                self.bsCenterPoint = bSquare['Object Center']
                #Bounding Box
                self.bsBoundinBox = bSquare['Bounding Box']
                json_blue_square_grabber()

                #Blue Triangle
                self.bTriangle = Blue['Triangle']
                self.btCenterPoint = bTriangle['Object Center']
                #Bounding Box
                self.btBoundinBox = bTriangle['Bounding Box']
                json_blue_triangle_grabber()

                #Ball data parsing
                self.ball_json = data['Ball']
                self.ball_coordinates = ball_json['Object Center']
                json_ball_grabber()

                #Corners data parsing
                self.corners = data['Corners']
                self.cornerBL = corners[0]
                self.cornerBR = corners[1]
                self.cornerTL = corners[2]
                self.cornerTR = corners[3]
                json_corner_grabber()
            rospy.spinOnce()
            rospy.sleep()

    def json_corner_grabber():
        corner = Map()
        corner.BotL.x = cornerBL['X']
        corner.BotL.y = cornerBL['Y']
        corner.BotR.x = cornerBR['X']
        corner.BotR.y = cornerBR['Y']
        corner.TopL.x = cornerTL['X']
        corner.TopL.y = cornerTL['Y']
        corner.TopR.x = cornerTR['X']
        corner.TopR.y = cornerTR['Y']
        pub_corner.publish(corner)

    def json_red_circle_grabber():
        player_rc = Rover()
        player_rc.center.x = rcCenterPoint['X']
        player_rc.center.y = rcCenterPoint['Y']
        player_rc.bound.TopL.x = rcBoundinBox['X Left']
        player_rc.bound.TopL.y = rcBoundinBox['Y Top']
        player_rc.bound.BotL.x = rcBoundinBox['X Left']
        player_rc.bound.BotL.y = rcBoundinBox['Y Bottom']
        player_rc.bound.TopR.x = rcBoundinBox['X Right']
        player_rc.bound.TopR.y = rcBoundinBox['Y Top']
        player_rc.bound.BotR.x = rcBoundinBox['X Right']
        player_rc.bound.BotR.y = rcBoundinBox['Y Bottom']
        pub_red_circle.publish(player_rc)

    def json_red_square_grabber():
        player_rs = Rover()
        player_rs.center.x = rsCenterPoint['X']
        player_rs.center.y = rsCenterPoint['Y']
        player_rs.bound.TopL.x = rsBoundinBox['X Left']
        player_rs.bound.TopL.y = rsBoundinBox['Y Top']
        player_rs.bound.BotL.x = rsBoundinBox['X Left']
        player_rs.bound.BotL.y = rsBoundinBox['Y Bottom']
        player_rs.bound.TopR.x = rsBoundinBox['X Right']
        player_rs.bound.TopR.y = rsBoundinBox['Y Top']
        player_rs.bound.BotR.x = rsBoundinBox['X Right']
        player_rs.bound.BotR.y = rsBoundinBox['Y Bottom']
        pub_red_square.publish(player_rs)

    def json_red_triangle_grabber():
        player_rt = Rover()
        player_rt.center.x = rtCenterPoint['X']
        player_rt.center.y = rtCenterPoint['Y']
        player_rt.bound.TopL.x = rtBoundinBox['X Left']
        player_rt.bound.TopL.y = rtBoundinBox['Y Top']
        player_rt.bound.BotL.x = rtBoundinBox['X Left']
        player_rt.bound.BotL.y = rtBoundinBox['Y Bottom']
        player_rt.bound.TopR.x = rtBoundinBox['X Right']
        player_rt.bound.TopR.y = rtBoundinBox['Y Top']
        player_rt.bound.BotR.x = rtBoundinBox['X Right']
        player_rt.bound.BotR.y = rtBoundinBox['Y Bottom']
        pub_red_triangle.publish(player_rt)

    def json_blue_circle_grabber():
        player_bc = Rover()
        player_bc.center.x = bcCenterPoint['X']
        player_bc.center.y = bcCenterPoint['Y']
        player_bc.bound.TopL.x = bcBoundinBox['X Left']
        player_bc.bound.TopL.y = bcBoundinBox['Y Top']
        player_bc.bound.BotL.x = bcBoundinBox['X Left']
        player_bc.bound.BotL.y = bcBoundinBox['Y Bottom']
        player_bc.bound.TopR.x = bcBoundinBox['X Right']
        player_bc.bound.TopR.y = bcBoundinBox['Y Top']
        player_bc.bound.BotR.x = bcBoundinBox['X Right']
        player_bc.bound.BotR.y = bcBoundinBox['Y Bottom']
        pub_blue_circle.publish(player_bc)

    def json_blue_square_grabber():
        player_bs = Rover()
        player_bs.center.x = bsCenterPoint['X']
        player_bs.center.y = bsCenterPoint['Y']
        player_bs.bound.TopL.x = bsBoundinBox['X Left']
        player_bs.bound.TopL.y = bsBoundinBox['Y Top']
        player_bs.bound.BotL.x = bsBoundinBox['X Left']
        player_bs.bound.BotL.y = bsBoundinBox['Y Bottom']
        player_bs.bound.TopR.x = bsBoundinBox['X Right']
        player_bs.bound.TopR.y = bsBoundinBox['Y Top']
        player_bs.bound.BotR.x = bsBoundinBox['X Right']
        player_bs.bound.BotR.y = bsBoundinBox['Y Bottom']
        pub_blue_square.publish(player_bs)

    def json_blue_triangle_grabber():
        player_bt = Rover()
        player_bt.center.x = btCenterPoint['X']
        player_bt.center.y = btCenterPoint['Y']
        player_bt.bound.TopL.x = btBoundinBox['X Left']
        player_bt.bound.TopL.y = btBoundinBox['Y Top']
        player_bt.bound.BotL.x = btBoundinBox['X Left']
        player_bt.bound.BotL.y = btBoundinBox['Y Bottom']
        player_bt.bound.TopR.x = btBoundinBox['X Right']
        player_bt.bound.TopR.y = btBoundinBox['Y Top']
        player_bt.bound.BotR.x = btBoundinBox['X Right']
        player_bt.bound.BotR.y = btBoundinBox['Y Bottom']
        pub_blue_triangle.publish(player_bt)

    def json_ball_grabber():
        ball = Vector2()
        ball.x = ball_coordinates['X']
        ball.y = ball_coordinates['Y']
        pub_ball.publish(ball)
