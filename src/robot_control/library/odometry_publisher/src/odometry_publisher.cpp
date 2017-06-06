#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
ros::Publisher odom_pub;
void get_odom(const nav_msgs::Odometry::ConstPtr &msg) {
  ros::Time current_time = ros::Time::now();
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(msg->pose.pose.position.x,
                                  msg->pose.pose.position.y,
                                  msg->pose.pose.position.z));
  tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                   msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  transform.setRotation(q);
  br.sendTransform(
      tf::StampedTransform(transform, current_time, "odom", "base_link"));
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";
  odom.pose.pose.position.x = msg->pose.pose.position.x;
  odom.pose.pose.position.y = msg->pose.pose.position.y;
  odom.pose.pose.position.z = msg->pose.pose.position.z;
  odom.pose.pose.orientation = msg->pose.pose.orientation;
  odom.child_frame_id = "base_link";
  odom_pub.publish(odom);
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "odometry_publisher");
  ros::NodeHandle n;
  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  ros::Subscriber sub = n.subscribe("/ground_truth_odom", 1, get_odom);
  ros::Rate r(100.0);
  ros::spin();
}
