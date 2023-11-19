#include <ros/ros.h>
#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

using namespace std;

string position_topic;

#define humanFrameID "human_detected"
#define fixedFrameID "base_link"

class PedestrianSpace {

  ros::NodeHandle nh_;
  ros::Subscriber pose_sub_;
  ros::Publisher marker_pub_;

public:
  PedestrianSpace() {
    pose_sub_ = nh_.subscribe(position_topic, 1000, &PedestrianSpace::poseCallback, this);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/pedestrian/marker", 1);
  }
  void poseCallback(const geometry_msgs::PoseStamped& pose_msg) {
    ros::Time rosTime = ros::Time::now();
    visualization_msgs::Marker marker;
    marker.header.frame_id = fixedFrameID;
    marker.header.stamp = rosTime;
    marker.ns = "basic_shapes";
    marker.id = 0;

    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = pose_msg.pose.position.x;
    marker.pose.position.y = pose_msg.pose.position.y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = pose_msg.pose.orientation.x;
    marker.pose.orientation.y = pose_msg.pose.orientation.y;
    marker.pose.orientation.z = pose_msg.pose.orientation.z;
    marker.pose.orientation.w = pose_msg.pose.orientation.w;
  
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.1;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    while (marker_pub_.getNumSubscribers() < 1) {
      if (!ros::ok()) {
        return ;
      }
      sleep(1);
    }
    marker_pub_.publish(marker);  
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "pedestrian_space");
  ros::NodeHandle nh;
  nh.getParam("/position_topic", position_topic);
  PedestrianSpace ic; 
  ros::spin ();
  return 0;
}
