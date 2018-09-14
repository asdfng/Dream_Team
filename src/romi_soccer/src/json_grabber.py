import rospy
import roslib
from romi_soccer.msg import Coordinates4
from rospy.numpy_msg import numpy_msg
class JSONGrabber():
    def __init__(self):
        init_pubs()
# Initializes publishers
    def init_pubs():
        rospy.loginfo('Initializing publisher...')
        pub = rospy.Publisher('mapper/raw_data',Coordinates4,queue_size=10)
        rospy.loginfo('Done.')
        rawJSON = Coordinates4()
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.json_grabber()
            pub.publish(rawJSON)
            rospy.spinOnce()
            rospy.sleep()

    def json_grabber():
        # Noah's code here

if __name__=='__main__':
    rospy.init_node('json_grabber')
    try:
        jason = JSONGrabber()
    except:
        rospy.ROSInterruptException: pass
