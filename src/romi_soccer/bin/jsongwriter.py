#! /usr/bin/env python
import rospy
from json_grabber import JSONGrabber
if __name__=='__main__':
    rospy.init_node('jsong_writer')
    try:
        rospy.loginfo('Initializing node...')
        jason = JSONGrabber()
    except rospy.ROSInterruptException:
            pass
