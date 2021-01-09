#include "point_cloud_range_image_convertor.h"

#include <pcl/range_image/range_image.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

PointCloudConvertor::PointCloudConvertor(
    pcl::visualization::RangeImageVisualizer& rangeImageWidget)
    : angularResolution(0.5f),
      maxAngleWidth(360.0f),   // 360.0 degree in radians
      maxAngleHeight(180.0f),  // 180.0 degree in radians
      sensorPose((Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f)),
      widget(rangeImageWidget) {
  sensorPose.rotate(
      Eigen::AngleAxisf(pcl::deg2rad(180.0f), Eigen::Vector3f::UnitZ()));
}

void PointCloudConvertor::RecvPointCloudCallBack(
    const sensor_msgs::PointCloud2::ConstPtr& pc2Msg) {
  // form ros pc2 to pcl xyz
  pcl::PointCloud<pcl::PointXYZ> pclCloud;
  pcl::fromROSMsg(*pc2Msg, pclCloud);

  // from xyz to range
  pcl::RangeImage rangeImage;
  {
    std::lock_guard<std::mutex> lock(lock_);
    rangeImage.createFromPointCloud(
        pclCloud, pcl::deg2rad(angularResolution), pcl::deg2rad(maxAngleWidth),
        pcl::deg2rad(maxAngleHeight), sensorPose, pcl::RangeImage::LASER_FRAME,
        0.00, 0.0f, 1);
  }

  // Show
  widget.showRangeImage(rangeImage);
}

void PointCloudConvertor::OnDynamicConfigChange(aiimooc_2_1_zhenyusha::CameraConfig& config,
                                                uint32_t level) {
  angularResolution = config.ang_res;
  maxAngleWidth = config.max_ang_w;
  maxAngleHeight = config.max_ang_h;
  std::lock_guard<std::mutex> lock(lock_);
  float theta = pcl::deg2rad(config.theta_ViewPort);
  sensorPose.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));
}