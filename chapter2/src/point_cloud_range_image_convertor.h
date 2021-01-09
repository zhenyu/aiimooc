#include <aiimooc_szy/CameraConfig.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Geometry>
#include <atomic>
#include <mutex>

#ifndef _ZHENYU_SHA_AIIMOOC_POINT_CLOUD_CONVERTOR_H
#define _ZHENYU_SHA_AIIMOOC_POINT_CLOUD_CONVERTOR_H
class PointCloudConvertor {
 public:
  PointCloudConvertor(pcl::visualization::RangeImageVisualizer&);
  void RecvPointCloudCallBack(const sensor_msgs::PointCloud2::ConstPtr&);
  void OnDynamicConfigChange(aiimooc_szy::CameraConfig&, uint32_t);

 private:
  std::atomic<float> angularResolution;
  std::atomic<float> maxAngleWidth;
  std::atomic<float> maxAngleHeight;
  std::mutex lock_;
  Eigen::Affine3f sensorPose;
  pcl::visualization::RangeImageVisualizer& widget;
};
#endif