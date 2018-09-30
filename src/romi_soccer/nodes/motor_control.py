#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

# Convert linear/angular velocity commands on cmd_vel topic to the right output for the encoders/motor pins or whatever
def callback(msg,team_name,shape_name):


if __name__=='__main__':
    # Initialize node to 'motor_control' (note: name can be overwritten in launch file)
    rospy.init_node('motor_control')
    # Get parameters for team_name and shape_name. I'm doing this so we can just call the same file to use for each node
    team_name = rospy.get_param('~team')
    shape_name = rospy.get_param('~shape')
    # Subscribe to the cmd_vel topic of the specific robot defined in the parameters
    rospy.Subscriber('%s/%s/cmd_vel' %team_name %shape_name,Twist,callback,team_name,shape_name)
