#include <string>
#include <ros/ros.h>
#include <boost/format.hpp>
#include <geometry_msgs/Twist.h>

class CommandPose {
private:
     ros::NodeHandle nh_;
     ros::Publisher pub_;
     tf::TransformListener listener_;
     std::string topic_;
     std::string frame_;
public:
     std::string team_name;
     std::string shape_name;
     CommandPose(ros::NodeHandle &nh) {
          this->nh_ = nh;
          nh_.getParam('~team',team_name);
          nh_.getParam('~shape',shape_name);
          this->topic_ = (boost::format('%s/%s/cmd_vel') % team_name % shape_name).str();
          this->frame_ = (boost::format('%s_%s') % team_name % shape_name).str();
          this->pub_ = nh_.advertise<geometry_msgs::Twist>(topic,1);
     }

     bool driveFwd(double dist) {
          listener_.waitForTransform('world',frame, ros::Time(0), ros::Duration(1.0));
          tf::StampedTransform start_transform;
          tf::StampedTransform current_transform;
          listener_.lookupTransform("world", frame, ros::Time(0), start_transform);
     }

};
