#1/usr/bin/env python
import json, urllib.request, time, rospy
from std_msg.msg import String

def talker():
        pub_all = rospy.Publisher('coordinates', String, queue_size=10)
        pub_ball = rospy.Publisher('coordinates', String, queue_size=10)
        pub_red_team = rospy.Publisher('coordinates', String, queue_size=10)
        pub_blue_team = rospy.Publisher('coordinates', String, queue_size=10)
        pub_corners = rospy.Publisher('coordinates', String, queue_size=10)
        rospy.init_node('JSON_scanner', anonymous=True)
        rate = rospy.Rate(10) #10 Hz
        while not rospy.is_shutdown():
            with urllib.request.urlopen('http://172.16.0.1:8001/FieldData/GetData') as response:
            source = response.read()
            data = json.loads(source.decode())
    #Ball data parsing
            ball = data['Ball']
            ball_coordinates = ball['Object Center']
            ball_coordinateX = ball_coordinates['X']
            ball_coordinateY = ball_coordinates['Y']
    #Corners data parsing
            corners = data['Corners']
            cornerBL = corners[0]
            cornerBR = corners[1]
            cornerTL = corners[2]
            cornerTR = corners[3]
            cornerBLX = cornerBL['X']
            cornerBLY = cornerBL['Y']
            cornerBRX = cornerBR['X']
            cornerBRY = cornerBR['Y']
            cornerTLX = cornerTL['X']
            cornerTLY = cornerTL['Y']
            cornerTRX = cornerTR['X']
            cornerTRY = cornerTR['Y']
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
            #publishing from node to topic: coodrinates
            pub_all.publish(data)
            pub_ball.publish(ball)
            pub_red_team.publish(Red)
            pub_red_team.publish(Blue)
            pub_corners.publish(corners)
            rate.sleep()
            
if __name__ == '__main__':
     try:
          talker()
     except rospy.ROSinterruptexecution:
        pass