#include <chapter2/CameraConfig.h> 
#include <Eigen/Geometry>
#include <pcl/visualization/range_image_visualizer.h>
#include <sensor_msgs/PointCloud2.h>


#ifndef _ZHENYU_SHA_AIIMOOC_POINT_CLOUD_CONVERTOR_H
#define _ZHENYU_SHA_AIIMOOC_POINT_CLOUD_CONVERTOR_H
class PointCloudConvertor {
 public:
  PointCloudConvertor(pcl::visualization::RangeImageVisualizer&);
  void RecvPointCloudCallBack(const sensor_msgs::PointCloud2::ConstPtr&);
  void OnDynamicConfigChange(chapter2::CameraConfig&, uint32_t);

 private:
  float angularResolution;
  float maxAngleWidth;
  float maxAngleHeight;
  Eigen::Affine3f sensorPose;
  float noiseLevel;
  float minRange;
  int borderSize;
  pcl::visualization::RangeImageVisualizer& widget;
};
#endif