#! /usr/bin/env python
import rospy
from image_mapper import ImageMapper
if __name__=='__main__':
    rospy.init_node('json_grabber')
    try:
        rospy.loginfo('Initializing node...')
        map = ImageMapper()
    except rospy.ROSInterruptException:
            pass
