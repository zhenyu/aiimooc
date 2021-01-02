#include "point_cloud_range_image_convertor.h"

#include <pcl/range_image/range_image.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

PointCloudConvertor::PointCloudConvertor() {
  angularResolution =
      (float)(1.0f * (M_PI / 180.0f));                //   1.0 degree in radians
  maxAngleWidth = (float)(360.0f * (M_PI / 180.0f));  // 360.0 degree in radians
  maxAngleHeight =
      (float)(180.0f * (M_PI / 180.0f));  // 180.0 degree in radians
  sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
  noiseLevel = 0.00;
  minRange = 0.0f;
  borderSize = 1;
}

void PointCloudConvertor::RecvPointCloudCallBack(const sensor_msgs::PointCloud2::ConstPtr& pc2_msg) {
  // form ros pc2 to pcl xyz
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl::fromROSMsg(*pc2_msg, pcl_cloud);
  // from xyz to range

  pcl::RangeImage rangeImage;
  rangeImage.createFromPointCloud(
      pcl_cloud, angularResolution, maxAngleWidth, maxAngleHeight, sensorPose,
      pcl::RangeImage::CAMERA_FRAME, noiseLevel, minRange, borderSize);
  ROS_INFO("in callback");
}