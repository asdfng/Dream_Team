#!/usr/bin/env python
import rospy, roslib, json, time
from romi_soccer.msg import Map
import urllib2

def json_corner_grabber():
    map = Map()
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        response = urllib2.urlopen('http://172.16.0.1:8001/FieldData/GetData')
        source = response.read()
        data = json.loads(source.decode())
        rospy.loginfo('It connected!')
        rospy.loginfo('Publishing raw data coordinates to topic...')
        map.header.time = rospy.Time.now()
        map.header.frame_id = 'world'
        #Corners data parsing
        corners = data['Corners']
        cornerTL = corners[0]
        cornerTR = corners[1]
        cornerBL = corners[2]
        cornerBR = corners[3]
        map.BotR.y = cornerBR['Y']
        map.BotL.x = cornerBL['X']
        map.BotL.y = cornerBL['Y']
        map.BotR.x = cornerBR['X']
        map.TopL.x = cornerTL['X']
        map.TopL.y = cornerTL['Y']
        map.TopR.x = cornerTR['X']
        map.TopR.y = cornerTR['Y']
        pub.publish(map)
        rospy.loginfo('Done.')
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('json_corner_grabber')
    pub = rospy.Publisher('mapper/raw_data/corners', Map, queue_size=10)
    try:
        json_corner_grabber()
    except rospy.ROSInterruptException:
        pass
