#include <string>
#include <memory>

#include <pcl/point_types.h> //点类型定义头文件
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h> //用于体素网格化的滤波类头文件
#include <pcl/filters/filter.h>

#ifndef _ZHENYU_SHA_AIIMOOC_ALIANER_H
#define _ZHENYU_SHA_AIIMOOC_ALIANER_H
//定义
typedef pcl::PointXYZ PointT;               // x,y,z点　
typedef pcl::PointCloud<PointT> PointCloud; //点云　申明pcl::PointXYZ数据

const std::string ICP_NAME = "icp";
const std::string NDT_NAME = "ndt";
class Aligner
{
public:
  // TODO smart pointer
  static Aligner *get_aligner(const std::string &name);
  ////////////////////////////////////////////////////////////////////////////////
  /** \brief Align a pair of PointCloud datasets and return the result
  * \param cloud_src the source PointCloud
  * \param cloud_tgt the target PointCloud
  * \param output the resultant aligned source PointCloud
  * \param final_transform the resultant transform between source and target
  */
  //实现匹配，其中参数有输入一组需要配准的点云，以及是否需要进行下采样，其他参数输出配准后的点云以及变换矩阵
  // target to source ==== output
  // final_transform = targetToSource
  virtual void align(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false) = 0;
  virtual ~Aligner() = default;
};
#endif