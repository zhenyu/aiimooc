#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
class PointClou
void chatterCallback(const sensor_msgs::PointCloud2::ConstPtr& pc2_msg) {
  // form ros pc2 to pcl xyz
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl::fromROSMsg(*pc2_msg, pcl_cloud);
  // from xyz
  float angularResolution = (float) (  1.0f * (M_PI/180.0f));  //   1.0 degree in radians
  float maxAngleWidth     = (float) (360.0f * (M_PI/180.0f));  // 360.0 degree in radians
  float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians
  Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
  pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
  float noiseLevel=0.00;
  float minRange = 0.0f;
  int borderSize = 1;
  
  pcl::RangeImage rangeImage;
  rangeImage.createFromPointCloud(pointCloud, angularResolution, maxAngleWidth, maxAngleHeight,
                                  sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
  // publish topic
} 

int main(int argc, char** argv) {
  ros::init(argc, argv, "pcl_to_range");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/rslidar_points", 1000, chatterCallback);
  ros::spin();
  return 0;
}