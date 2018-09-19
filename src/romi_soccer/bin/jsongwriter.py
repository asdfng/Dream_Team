#! /usr/bin/env python
import rospy
import romi_soccer.json_grabber
if __name__=='__main__':
    rospy.init_node('jsong_writer')
    try:
        rospy.loginfo('Initializing node...')
        jason = JSONGrabber()
    except rospy.ROSInterruptException:
            pass
