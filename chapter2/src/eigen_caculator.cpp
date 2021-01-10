#include <cmath>

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

#include "eigen_caculator.h"

static void Compute(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::PointXYZ& point, const int index, const int K ){
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

  kdtree.setInputCloud(cloud);
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);
  if ( kdtree.nearestKSearch (point, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
  {
    pointIdxNKNSearch.push_back(index);
    Eigen::Matrix <double, 4, 1> centroid;
  }

}
void EigenCaculator::RecvPointCloudCallBack(
    const sensor_msgs::PointCloud2::ConstPtr& pc2Msg) {
  // form ros pc2 to pcl xyz
  pcl::PointCloud<pcl::PointXYZ> pclCloud;
  pcl::fromROSMsg(*pc2Msg, pclCloud);
  static bool first = true;
  if(first){
      size_t len = pclCloud.points.size();
      const pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ptr (&pclCloud);
      for(int i=0;i<len;i++){
        pcl::PointXYZ& point =  pclCloud.points[i];
        if(std::isfinite(point.x)&&std::isfinite(point.y)&&std::isfinite(point.z)) {
            Compute(pcl_ptr, point, i, 20);
        }
      }
  } else
  {
      ROS_INFO("ignore");
  }
  
}

