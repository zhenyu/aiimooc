#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

void chatterCallback(const sensor_msgs::PointCloud2::ConstPtr& pc2_msg) {
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl::fromROSMsg(*pc2_msg, pcl_cloud);
  ROS_INFO("converted to pcl format");
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pcl_to_range");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/rslidar_points", 1000, chatterCallback);
  ros::spin();
  return 0;
}