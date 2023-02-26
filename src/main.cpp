#include <ros/ros.h>
#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

using namespace std;

/* Global variables */
#define humanFrameID "human_detected"
#define fixedFrameID "camera_link"

ros::Publisher marker_pub;
ros::Subscriber pose_sub;
  
void dpsCallback(const geometry_msgs::PoseStamped& pose_msg) {
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
  marker.pose.position.z = pose_msg.pose.position.z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  float Velocity = 0.5;
  float Time = 0.4;
  float back = 0.5;
  float side = 0.5;
  float alpha = 1.0;
  float front = back + alpha*Velocity*Time;

  float a = (front + back)/(2);
  float b = side; /*revisar esto*/
  float c = (front - back)/(2);
  
  marker.scale.x = a/2;
  marker.scale.y = b/2;
  marker.scale.z = 0.01;

  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  while (marker_pub.getNumSubscribers() < 1) {
    if (!ros::ok()) {
        return ;
      }
      sleep(1);
    }
    marker_pub.publish(marker); 
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "Human_dection");
  ros::NodeHandle nh;

  pose_sub = nh.subscribe("human/position", 1000, dpsCallback);
  marker_pub = nh.advertise<visualization_msgs::Marker>("human/marker", 1);
  
  while (ros::ok()) {
    ros::spinOnce();
  }
}
