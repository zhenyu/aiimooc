#include <cmath>

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

#include "eigen_caculator.h"

static void Compute(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::PointXYZ& point, const int index, const int K, std::ofstream & fs){
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

  kdtree.setInputCloud(cloud);
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);
  if ( kdtree.nearestKSearch (point, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
  {
    pointIdxNKNSearch.push_back(index);
    Eigen::Matrix <double, 4, 1> centroid;
    pcl::compute3DCentroid (*cloud, pointIdxNKNSearch, centroid);
    Eigen::Matrix< double, 3, 3 > covariance_matrix;
    pcl::computeCovarianceMatrix (*cloud, pointIdxNKNSearch, centroid, covariance_matrix);
    Eigen::Matrix3d eigenvals;
    Eigen::Vector3d eigenvector;
    pcl::eigen33 (covariance_matrix, eigenvals, eigenvector);
    Eigen::Vector3d::Index maxRow, minRow, maxCol, minCol;
    eigenvals.minCoeff(&minRow, &minCol);
    eigenvals.maxCoeff(&maxRow, &maxCol);
    ROS_DEBUG_STREAM("max="<<maxRow<<", min="<<minRow<<", midle="<<3-minRow-maxRow);
    double l1 = eigenvals(maxRow);
    double l3 = eigenvals(minRow);
    double l2 =  eigenvals( minRow==maxRow? maxRow: 3-minRow-maxRow);
    double l = l1+l2+l3;
    double e1 = l1/l;
    double e2 = l2/l;
    double e3 = l3/l;
    fs<<"====== number "<<index<<" point============="<<std::endl;
    fs<<"L="<<(l1-l2)/l1<<std::endl;
    fs<<"p="<<(l2-l3)/l1<<std::endl;
    fs<<"S="<<l3/l1<<std::endl;
    fs<<"O="<<3*pow(e1*e2*e3, double(1.0/3.0))<<std::endl;
    fs<<"E="<<-1*(e1*log(e1)+e2*log(e2)+e3*log(e3))<<std::endl;
    fs<<"C="<<3*e3<<std::endl<<std::endl;
  }

}
void EigenCaculator::RecvPointCloudCallBack(
    const sensor_msgs::PointCloud2::ConstPtr& pc2Msg) {
  // form ros pc2 to pcl xyz
  pcl::PointCloud<pcl::PointXYZ> pclCloud;
  pcl::fromROSMsg(*pc2Msg, pclCloud);
  static bool first = true;
  if(first){
      auto len = pclCloud.points.size();
      ROS_INFO_STREAM(len << " points\n");
      const pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ptr (&pclCloud);
      std::ofstream fs("eigentfeature.txt");
      for(int i=0;i<len;i++){
        pcl::PointXYZ& point =  pclCloud.points[i];
        if(std::isfinite(point.x)&&std::isfinite(point.y)&&std::isfinite(point.z)) {
            Compute(pcl_ptr, point, i, 20, fs);
        }
      }
      fs.close();
      ROS_INFO("first frame done\n");
      first= false;
  } else
  {
      ROS_INFO("ignore\n");
  }
  
}

