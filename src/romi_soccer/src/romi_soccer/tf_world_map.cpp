#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
// #include <std_msgs/Float64.h>
#include <romi_soccer/Homography.h>
#include <romi_soccer/Map.h>
// #include <tf2/LinearMath/Quaternion.h>
class WorldToMap {
public:
     // std_msgs::Float64 u;
     // std_msgs::Float64 v;
     bool grabbed;
     double u,v;
     double tranx_x, trans_y;
     double center_u, center_v;
     double center_x, center_y;
     WorldToMap() {
          this->grabbed = 0;
          this->center_x = 0;
          this->center_y = 0;
     }
     void pixCallback(const romi_soccer::Map &msg) {
          this->center_u = (msg.TopL.x + msg.TopR.x + msg.BotL.x + msg.BotR.x);
          this->center_v = (msg.TopL.y + msg.TopR.y + msg.BotL.y + msg.BotR.y);
          this->grabbed = 1;
     }
     void callback(const romi_soccer::Homography &msg) {
          if (grabbed) {
               static tf2_ros::TransformBroadcaster br;
               tf2::Quaternion quat(0,0,0,1);
               this->trans_x = ((msg.q11*center_u+msg.q12*center_v+msg.q13)/(msg.q31*center_u+msg.q32*center_v+msg.q33));
               this->trans_y = ((msg.q21*center_u+msg.q22*center_v+msg.q23)/(msg.q31*center_u+msg.q32*center_v+msg.q33));
               tf2::Vector3 translation(trans_x, trans_y, 0);
               tf2::Transform opt_prime(quat, translation);
               geometry_msgs::TransformStamped transformStamped;
               transformStamped.header.stamp = ros::Time::now();
               transformStamped.header.frame_id = "world";
               transformStamped.child_frame_id = "map";
               transformStamped.transform = tf2::toMsg(opt_prime);
               br.sendTransform(transformStamped);
          } else {

          }
     }
};

int main(int argc, char **argv) {
     ros::init(argc, argv, 'br_world_to_map');
     ros::NodeHandle nh;
     ros::Subscriber sub = nh.subscribe('mapper/Homography',10,&callback);
     ros::Subscriber sub_corner = nh.subscribe('mapper/raw_data/corners',10,&callback);
     tf_wm = WorldToMap();
     ros::spin();
}
