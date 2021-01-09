
#include <sensor_msgs/PointCloud2.h>

#ifndef _ZHENYU_SHA_AIIMOOC_POINT_EIGEN_CACULATOR_H
#define _ZHENYU_SHA_AIIMOOC_POINT_EIGEN_CACULATOR_H
class EigenCaculator {
 public:  
  void RecvPointCloudCallBack(const sensor_msgs::PointCloud2::ConstPtr&);
};
#endif