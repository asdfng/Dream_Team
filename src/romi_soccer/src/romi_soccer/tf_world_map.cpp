#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
// #include <std_msgs/Float64.h>
#include <romi_soccer/Homography.h>
#include <romi_soccer/Map.h>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco.hpp>
#include <vector>
void pixCallback(const romi_soccer::Map &msg) {
     std::vector<cv::Mat> rotation, translation, normals;
     std::vector<cv::Point2f> corners_img, corners_tab;
     cv::Point2f corner_TopL(msg.TopL.x,msg.TopL.y);
     corners_img.push_back(corner_TopL);
     cv::Point2f p0(0,0);
     corners_tab.push_back(p0);
     cv::Point2f corner_TopR(msg.TopR.x,msg.TopR.y);
     corners_img.push_back(corner_TopR);
     cv::Point2f p1(0,8);
     corners_tab.push_back(p1);
     cv::Point2f corner_BotL(msg.BotL.x,msg.BotL.y);
     corners_img.push_back(corner_BotL);
     cv::Point2f p2(4,0);
     corners_tab.push_back(p2);
     cv::Point2f corner_BotR(msg.BotR.x,msg.BotR.y);
     corners_img.push_back(corner_BotR);
     cv::Point2f p3(4,8);
     corners_tab.push_back(p3);

     cv::Mat homography = cv::findHomography(corners_img,corners_tab);
     if (H) {
          ROS_INFO('It worked! H was found.');
     }
     int solutions = cv::decomposeHomographyMat(homography,cameraMat,rotation,translation,normals);
     

}

int main(int argc, char **argv) {
     ros::init(argc, argv, 'br_world_to_map');
     ros::NodeHandle nh;
     ros::Subscriber sub = nh.subscribe('mapper/Homography',10,&callback);
     ros::Subscriber sub_corner = nh.subscribe('mapper/raw_data/corners',10,&callback);
     tf_wm = WorldToMap();
     ros::spin();
}
