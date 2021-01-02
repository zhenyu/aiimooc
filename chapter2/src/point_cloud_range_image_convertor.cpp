#include "point_cloud_range_image_convertor.h"

#include <pcl/range_image/range_image.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

PointCloudConvertor::PointCloudConvertor(
    pcl::visualization::RangeImageVisualizer& rangeImageWidget)
    : angularResolution(
          (float)(0.2f * (M_PI / 180.0f))),  //   1.0 degree in radians
      maxAngleWidth(
          (float)(360.0f * (M_PI / 180.0f))),  // 360.0 degree in radians
      maxAngleHeight((float)(180.0f * (M_PI / 180.0f))),// 180.0 degree in radians
      sensorPose((Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f)),
      noiseLevel(0.00),
      minRange (0.0f),
      borderSize(1),
      widget(rangeImageWidget) {}

void PointCloudConvertor::RecvPointCloudCallBack(
    const sensor_msgs::PointCloud2::ConstPtr& pc2Msg) {
  // form ros pc2 to pcl xyz
  pcl::PointCloud<pcl::PointXYZ> pclCloud;
  pcl::fromROSMsg(*pc2Msg, pclCloud);

  // from xyz to range
  pcl::RangeImage rangeImage;
  rangeImage.createFromPointCloud(
      pclCloud, angularResolution, maxAngleWidth, maxAngleHeight, sensorPose,
      pcl::RangeImage::LASER_FRAME, noiseLevel, minRange, borderSize);

  // Show
  widget.showRangeImage(rangeImage);
}

void PointCloudConvertor::OnDynamicConfigChange(chapter2::CameraConfig& config,
                                                uint32_t level) {}