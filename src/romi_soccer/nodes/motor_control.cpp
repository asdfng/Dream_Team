#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include <string>
#include <Romi32U4Encoders.h>


// Noah, use this or the Python one to convert a linear/angular velocity command to an output for the romi using the encoders
void callback(const geometry_msgs::Twist::ConstPtr &msg){
     if (Romi32U4Encoders::getCountsLeft == )
}


int main(int argc, char **argv){
     ros::init(argc, argv, 'motor_control');
     ros::NodeHandle nh;
     std::string team_name;
     std::string shape_name;
     nh.getParam('team', team_name);
     nh.getParam('shape', shape_name);
     ros::Subscriber sub = nh.subscribe('%s_team/%s/cmd_vel' % team_name % shape_name, 1000, &callback);
     ros::spin()
}
