#!/usr/bin/env python
import rospy, roslib, json, time, tf2_ros, tf_conversions, math
import urllib2
import geometry_msgs.msg

def json_team_grabber():
    rover = geometry_msgs.msg.PoseStamped()
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        response = urllib2.urlopen('http://192.168.137.1:8001/FieldData/GetData')
        source = response.read()
        data = json.loads(source.decode())
        rospy.loginfo('It connected!')
        #Team data parsing
        team = data['%s Team Data' % team_name]
        #Shape data parsing
        shape = team['%s' % shape_name]
        center_shape = shape['Object Center']
        rospy.loginfo('Received %s %s coordinates from JSON.' % (team_name,shape_name))
        rospy.loginfo('Publishing raw data coordinates to topic...')
        rover.header.stamp = rospy.Time.now()
        rover.header.frame_id = 'world'
        rover.pose.position.x = center_shape['X']
        rover.pose.position.y = center_shape['Y']
        # 2D Rotation uses JSON data for now as placeholder, will replace with
        # IMU data later
        th = shape['Orientation']
        angle = tf_conversions.transformations.quaternion_from_euler(0,0,th)
        rover.pose.orientation.x = angle[0]
        rover.pose.orientation.y = angle[1]
        rover.pose.orientation.z = angle[2]
        rover.pose.orientation.w = angle[3]

        pub.publish(rover)
        rospy.loginfo('Done.')
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('json_team_grabber')
    team_name_lc = rospy.get_param('~team')
    team_name = team_name_lc.capitalize()
    shape_name_lc = rospy.get_param('~shape')
    shape_name = shape_name_lc.capitalize()
    if rospy.has_param('~robot_name'):
        robot_name = rospy.get_param('~robot_name')
        pub = rospy.Publisher('/%s_%s/%s/raw_pose' % (team_name_lc,shape_name_lc,robot_name), geometry_msgs.msg.PoseStamped, queue_size=10)
    else:
        pub = rospy.Publisher('/%s_%s/raw_pose' % (team_name_lc,shape_name_lc), geometry_msgs.msg.PoseStamped, queue_size=10)
    try:
        json_team_grabber()
    except rospy.ROSInterruptException:
        pass
