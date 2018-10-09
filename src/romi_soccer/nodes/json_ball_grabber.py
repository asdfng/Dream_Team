#!/usr/bin/env python
import rospy, roslib, json, time
from geometry_msgs.msg import Vector3Stamped
import urllib2

def json_ball_grabber():
    ball = PoseStamped()
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        response = urllib2.urlopen('http://172.16.0.1:8001/FieldData/GetData')
        source = response.read()
        data = json.loads(source.decode())
        rospy.loginfo('It connected!')
        rospy.loginfo('Publishing raw data coordinates to topic...')
        ball.header.stamp = rospy.Time.now()
        ball.header.frame_id = 'world'
        # Ball data parsing
        ball_json = data['Ball']
        ball_json_coordinates = ball_json['Object Center']
        ball.vector.x = ball_json_coordinates['X']
        ball.vector.y = ball_json_coordinates['Y']
        ball.vector.z = 0
        rospy.loginfo('Done.')
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('json_ball_grabber')
    subject = rospy.get_param('~subject')
    pub = rospy.Publisher('ball/raw_pose', Vector3Stamped, queue_size=10)
    try:
        json_ball_grabber()
    except rospy.ROSInterruptException:
        pass
