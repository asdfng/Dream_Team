#! /usr/bin/env python
import rospy, roslib, json, time
from romi_soccer.msg import Map, Vector2, Rover
import urllib2

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
            response = urllib2.urlopen('http://172.16.0.1:8001/FieldData/GetData')
            source = response.read()
            data = json.loads(source.decode())
            #Red Team data parsing
            self.Red = data['Red Team Data']
            #Blue Team data parsing
            self.Blue = data['Blue Team Data']

            #Red Circle
            self.rCircle = self.Red['Circle']
            self.rcCenterPoint = self.rCircle['Object Center']
            #Bounding Box
            self.rcBoundinBox = self.rCircle['Bounding Box']
            self.json_red_circle_grabber()

            #Red Square
            self.rSquare = self.Red['Square']
            self.rsCenterPoint = self.rSquare['Object Center']
            #Bounding Box
            self.rsBoundinBox = self.rSquare['Bounding Box']
            self.json_red_square_grabber()

            #Red Triangle
            self.rTriangle = self.Red['Triangle']
            self.rtCenterPoint = self.rTriangle['Object Center']
            #Bounding Btox
            self.rtBoundinBox = self.rTriangle['Bounding Box']
            self.json_red_triangle_grabber()

            #Blue Circle
            self.bCircle = self.Blue['Circle']
            self.bcCenterPoint = self.bCircle['Object Center']
            #Bounding Box
            self.bcBoundinBox = self.bCircle['Bounding Box']
            self.json_blue_circle_grabber()

            #Blue Square
            self.bSquare = self.Blue['Square']
            self.bsCenterPoint = self.bSquare['Object Center']
            #Bounding Box
            self.bsBoundinBox = self.bSquare['Bounding Box']
            self.json_blue_square_grabber()

            #Blue Triangle
            self.bTriangle = self.Blue['Triangle']
            self.btCenterPoint = self.bTriangle['Object Center']
            #Bounding Box
            self.btBoundinBox = self.bTriangle['Bounding Box']
            self.json_blue_triangle_grabber()

            #Ball data parsing
            self.ball_json = data['Ball']
            self.ball_coordinates = self.ball_json['Object Center']
            self.json_ball_grabber()

            #Corners data parsing
            self.corners = data['Corners']
            self.cornerBL = self.corners[0]
            self.cornerBR = self.corners[1]
            self.cornerTL = self.corners[2]
            self.cornerTR = self.corners[3]
            self.json_corner_grabber()

            # rospy.spin()
            rate.sleep()

    def json_corner_grabber(self):
        corner = Map()
        corner.BotL.x = self.cornerBL['X']
        corner.BotL.y = self.cornerBL['Y']
        corner.BotR.x = self.cornerBR['X']
        corner.BotR.y = self.cornerBR['Y']
        corner.TopL.x = self.cornerTL['X']
        corner.TopL.y = self.cornerTL['Y']
        corner.TopR.x = self.cornerTR['X']
        corner.TopR.y = self.cornerTR['Y']
        self.pub_corner.publish(corner)

    def json_red_circle_grabber(self):
        player_rc = Rover()
        player_rc.center.x = self.rcCenterPoint['X']
        player_rc.center.y = self.rcCenterPoint['Y']
        player_rc.bound.TopL.x = self.rcBoundinBox['X Left']
        player_rc.bound.TopL.y = self.rcBoundinBox['Y Top']
        player_rc.bound.BotL.x = self.rcBoundinBox['X Left']
        player_rc.bound.BotL.y = self.rcBoundinBox['Y Bottom']
        player_rc.bound.TopR.x = self.rcBoundinBox['X Right']
        player_rc.bound.TopR.y = self.rcBoundinBox['Y Top']
        player_rc.bound.BotR.x = self.rcBoundinBox['X Right']
        player_rc.bound.BotR.y = self.rcBoundinBox['Y Bottom']
        self.pub_red_circle.publish(player_rc)

    def json_red_square_grabber(self):
        player_rs = Rover()
        player_rs.center.x = self.rsCenterPoint['X']
        player_rs.center.y = self.rsCenterPoint['Y']
        player_rs.bound.TopL.x = self.rsBoundinBox['X Left']
        player_rs.bound.TopL.y = self.rsBoundinBox['Y Top']
        player_rs.bound.BotL.x = self.rsBoundinBox['X Left']
        player_rs.bound.BotL.y = self.rsBoundinBox['Y Bottom']
        player_rs.bound.TopR.x = self.rsBoundinBox['X Right']
        player_rs.bound.TopR.y = self.rsBoundinBox['Y Top']
        player_rs.bound.BotR.x = self.rsBoundinBox['X Right']
        player_rs.bound.BotR.y = self.rsBoundinBox['Y Bottom']
        self.pub_red_square.publish(player_rs)

    def json_red_triangle_grabber(self):
        player_rt = Rover()
        player_rt.center.x = self.rtCenterPoint['X']
        player_rt.center.y = self.rtCenterPoint['Y']
        player_rt.bound.TopL.x = self.rtBoundinBox['X Left']
        player_rt.bound.TopL.y = self.rtBoundinBox['Y Top']
        player_rt.bound.BotL.x = self.rtBoundinBox['X Left']
        player_rt.bound.BotL.y = self.rtBoundinBox['Y Bottom']
        player_rt.bound.TopR.x = self.rtBoundinBox['X Right']
        player_rt.bound.TopR.y = self.rtBoundinBox['Y Top']
        player_rt.bound.BotR.x = self.rtBoundinBox['X Right']
        player_rt.bound.BotR.y = self.rtBoundinBox['Y Bottom']
        self.pub_red_triangle.publish(player_rt)

    def json_blue_circle_grabber(self):
        player_bc = Rover()
        player_bc.center.x = self.bcCenterPoint['X']
        player_bc.center.y = self.bcCenterPoint['Y']
        player_bc.bound.TopL.x = self.bcBoundinBox['X Left']
        player_bc.bound.TopL.y = self.bcBoundinBox['Y Top']
        player_bc.bound.BotL.x = self.bcBoundinBox['X Left']
        player_bc.bound.BotL.y = self.bcBoundinBox['Y Bottom']
        player_bc.bound.TopR.x = self.bcBoundinBox['X Right']
        player_bc.bound.TopR.y = self.bcBoundinBox['Y Top']
        player_bc.bound.BotR.x = self.bcBoundinBox['X Right']
        player_bc.bound.BotR.y = self.bcBoundinBox['Y Bottom']
        self.pub_blue_circle.publish(player_bc)

    def json_blue_square_grabber(self):
        player_bs = Rover()
        player_bs.center.x = self.bsCenterPoint['X']
        player_bs.center.y = self.bsCenterPoint['Y']
        player_bs.bound.TopL.x = self.bsBoundinBox['X Left']
        player_bs.bound.TopL.y = self.bsBoundinBox['Y Top']
        player_bs.bound.BotL.x = self.bsBoundinBox['X Left']
        player_bs.bound.BotL.y = self.bsBoundinBox['Y Bottom']
        player_bs.bound.TopR.x = self.bsBoundinBox['X Right']
        player_bs.bound.TopR.y = self.bsBoundinBox['Y Top']
        player_bs.bound.BotR.x = self.bsBoundinBox['X Right']
        player_bs.bound.BotR.y = self.bsBoundinBox['Y Bottom']
        self.pub_blue_square.publish(player_bs)

    def json_blue_triangle_grabber(self):
        player_bt = Rover()
        player_bt.center.x = self.btCenterPoint['X']
        player_bt.center.y = self.btCenterPoint['Y']
        player_bt.bound.TopL.x = self.btBoundinBox['X Left']
        player_bt.bound.TopL.y = self.btBoundinBox['Y Top']
        player_bt.bound.BotL.x = self.btBoundinBox['X Left']
        player_bt.bound.BotL.y = self.btBoundinBox['Y Bottom']
        player_bt.bound.TopR.x = self.btBoundinBox['X Right']
        player_bt.bound.TopR.y = self.btBoundinBox['Y Top']
        player_bt.bound.BotR.x = self.btBoundinBox['X Right']
        player_bt.bound.BotR.y = self.btBoundinBox['Y Bottom']
        self.pub_blue_triangle.publish(player_bt)

    def json_ball_grabber(self):
        ball = Vector2()
        ball.x = self.ball_coordinates['X']
        ball.y = self.ball_coordinates['Y']
        self.pub_ball.publish(ball)
