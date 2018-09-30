#!/usr/bin/env python
import rospy, roslib, json, time, tf2_ros, tf_conversions, math
import urllib2
import geometry_msgs.msg

def json_team_grabber():
    rover = Rover()
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        response = urllib2.urlopen('http://172.16.0.1:8001/FieldData/GetData')
        source = response.read()
        data = json.loads(source.decode())
        rospy.loginfo('It connected!')
        #Team data parsing
        team = data['%s Team Data' % team_name]
        #Shape data parsing
        shape = team['%s' % shape_name]
        center_shape = shape['Object Center']
        rospy.loginfo('Received %s %s coordinates from JSON.' % team_name % shape_name)
        rospy.loginfo('Publishing raw data coordinates to topic...')
        rover.header.time = rospy.Time.now()
        rover.header.frame_id = 'world'
        rover.pose.position.x = center_shape['X']
        rover.pose.position.y = center_shape['Y']
        th = shape['Orientation']
        x = center_shape['X']
        y = center_shape['Y']
        # 2D Rotation uses JSON data for now as placeholder, will replace with
        # IMU data later
        rover.pose.orientation.x = x*math.cos(th)-y*math.sin(th)
        rover.pose.orientation.y = x*math.sin(th)+y*math.cos(th)
        pub.publish(rover)
        rospy.loginfo('Done.')
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('json_team_grabber')
    team_name = rospy.get_param('~team')
    shape_name = rospy.get_param('~shape')
    pub = rospy.Publisher('mapper/raw_data/%s/%s' %team_name %shape_name, geometry_msgs.msg.PoseStamped, queue_size=10)
    try:
        json_team_grabber()
    except rospy.ROSInterruptException:
        pass
