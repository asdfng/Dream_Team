import rospy
import roslib
from romi_soccer.msg import Map, Vector2
from rospy.numpy_msg import numpy_msg
import json, urllib.request, time
class JSONGrabber():
    def __init__(self):
        init_pubs()
        self.corner = Map()
        with urllib.request.urlopen('http://172.16.0.1:8001/FieldData/GetData') as response:
        self.source = response.read()
        self.data = json.loads(source.decode())
# Initializes publishers
    def init_pubs():
        rospy.loginfo('Initializing publisher...')
        pub_corner = rospy.Publisher('mapper/raw_data/corners',Map,queue_size=10)
        pub_ball = rospy.Publisher('mapper/raw_data/ball',Vector2,queue_size=10)
        pub_ = rospy.Publisher('mapper/raw_data/corners',Map,queue_size=10)
        rospy.loginfo('Done.')
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            # self.json_grabber()
            pub.publish(corners)
            rospy.spinOnce()
            rospy.sleep()

    def json_grabber():
        while not rospy.is_shutdown:
            with urllib.request.urlopen('http://172.16.0.1:8001/FieldData/GetData') as response:
            self.source = response.read()
            self.data = json.loads(source.decode())
            json_ball_grabber()
            json_corner_grabber()
            rospy.spinOnce()
            rospy.sleep()

    def json_corner_grabber():
        #Corners data parsing
        corners = data['Corners']
        cornerBL = corners[0]
        cornerBR = corners[1]
        cornerTL = corners[2]
        cornerTR = corners[3]
        corner.BotL.x = cornerBL['X']
        corner.BotL.y = cornerBL['Y']
        corner.BotR.x = cornerBR['X']
        corner.BotR.y = cornerBR['Y']
        corner.TopL.x = cornerTL['X']
        corner.TopL.y = cornerTL['Y']
        corner.TopR.x = cornerTR['X']
        corner.TopR.y = cornerTR['Y']
        #Red Team data parsing
         Red = data['Red Team Data']
        #Red Circle
        rCircle = Red['Circle']
        rcCenterPoint = rCircle['Object Center']
        #Coordinates
        rccpX = rcCenterPoint['X']
        rccpY = rcCenterPoint['Y']
        #Bounding Box
        rcBoundinBox = rCircle['Bounding Box']
        rcbbXL = rcBoundinBox['X Left']
        rcbbYT = rcBoundinBox['Y Top']
        rcbbXR = rcBoundinBox['X Right']
        rcbbYB = rcBoundinBox['Y Bottom']
        #Area
        rcArea = rCircle['Area']
        #Red Square
        rSquare = Red['Square']
        rsCenterPoint = rSquare['Object Center']
        #Coordinates
        rscpX = rsCenterPoint['X']
        rscpY = rsCenterPoint['Y']
        #Bounding Box
        rsBoundinBox = rSquare['Bounding Box']
        rsbbXL = rsBoundinBox['X Left']
        rsbbYT = rsBoundinBox['Y Top']
        rsbbXR = rsBoundinBox['X Right']
        rsbbYB = rsBoundinBox['Y Bottom']
        #Area
        rsArea = rSquare['Area']
        #Red Triangle
        rTriangle = Red['Triangle']
        rtCenterPoint = rTriangle['Object Center']
        #Coordinates
        rtcpX = rtCenterPoint['X']
        rtcpY = rtCenterPoint['Y']
        #Bounding Btox
        rtBoundinBox = rTriangle['Bounding Box']
        rtbbXL = rtBoundinBox['X Left']
        rtbbYT = rtBoundinBox['Y Top']
        rtbbXR = rtBoundinBox['X Right']
        rtbbYB = rtBoundinBox['Y Bottom']
        #Area
        rtArea = rTriangle['Area']
        #Write Red team data to ROS message (.msg)
        #Blue Team data parsing
        Blue = data['Blue Team Data']
        #Blue Circle
        bCircle = Blue['Circle']
        bcCenterPoint = bCircle['Object Center']
        #Coordinates
        bccpX = bcCenterPoint['X']
        bccpY = bcCenterPoint['Y']
        #Bounding Box
        bcBoundinBox = bCircle['Bounding Box']
        bcbbXL = bcBoundinBox['X Left']
        bcbbYT = bcBoundinBox['Y Top']
        bcbbXR = bcBoundinBox['X Right']
        bcbbYB = bcBoundinBox['Y Bottom']
        #Area
        bcArea = bCircle['Area']
        #Blue Square
        bSquare = Blue['Square']
        bsCenterPoint = bSquare['Object Center']
        #Coordinates
        bscpX = bsCenterPoint['X']
        bscpY = bsCenterPoint['Y']
        #Bounding Box
        bsBoundinBox = bSquare['Bounding Box']
        bsbbXL = bsBoundinBox['X Left']
        bsbbYT = bsBoundinBox['Y Top']
        bsbbXR = bsBoundinBox['X Right']
        bsbbYB = bsBoundinBox['Y Bottom']
        #Area
        bsArea = bSquare['Area']
        #Blue Triangle
        bTriangle = Blue['Triangle']
        btCenterPoint = bTriangle['Object Center']
        #Coordinates
        btcpX = btCenterPoint['X']
        btcpY = btCenterPoint['Y']
        #Bounding Box
        btBoundinBox = bSquare['Bounding Box']
        btbbXL = btBoundinBox['X Left']
        btbbYT = btBoundinBox['Y Top']
        btbbXR = btBoundinBox['X Right']
        btbbYB = btBoundinBox['Y Bottom']
        #Area
        btArea = bTriangle['Area']

def json_ball_grabber():
    #Ball data parsing
    ball = data['Ball']
    ball_coordinates = ball['Object Center']
    ball_coordinateX = ball_coordinates['X']
    ball_coordinateY = ball_coordinates['Y']


if __name__=='__main__':
    rospy.init_node('json_grabber')
    try:
        jason = JSONGrabber()
    except:
        rospy.ROSInterruptException: pass
