#include <pcl_conversions/pcl_conversions.h>

#include "eigen_caculator.h"

void EigenCaculator::RecvPointCloudCallBack(
    const sensor_msgs::PointCloud2::ConstPtr& pc2Msg) {
  // form ros pc2 to pcl xyz
  pcl::PointCloud<pcl::PointXYZ> pclCloud;
  pcl::fromROSMsg(*pc2Msg, pclCloud);

}

