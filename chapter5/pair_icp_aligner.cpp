             //滤波相关头文件
#include <pcl/features/normal_3d.h>           //法线特征头文件
#include <pcl/registration/icp.h>             //ICP类相关头文件
#include <pcl/registration/icp_nl.h>          //非线性ICP 相关头文件
#include <pcl/point_representation.h>         //点表示相关的头文件

#include "pair_icp_aligner.h"

typedef pcl::PointNormal PointNormalT;      //　x,y,z＋法向量＋曲率　点
// float x,y,z;   float normal[3] ,curvature ;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals; //带有法向量的点云

// 以< x, y, z, curvature >形式定义一个新的点表示 x,y,z ＋曲率
class MyPointRepresentation : public pcl::PointRepresentation<PointNormalT>
{
  using pcl::PointRepresentation<PointNormalT>::nr_dimensions_; //维度
public:
  MyPointRepresentation()
  {
    nr_dimensions_ = 4; //定义点的维度
  }

  // 重载copyToFloatArray方法将点转化为四维数组
  virtual void copyToFloatArray(const PointNormalT &p, float *out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature; //曲率
  }
};

void PairICPAligner::align(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample) {
  // Downsample for consistency and speed
  // \note enable this for large datasets
  PointCloud::Ptr src(new PointCloud); //存储滤波后的源点云
  PointCloud::Ptr tgt(new PointCloud); //存储滤波后的目标点云
  pcl::VoxelGrid<PointT> grid;         //体素格滤波器 滤波处理对象
  if (downsample)
  {
    grid.setLeafSize(0.05, 0.05, 0.05); //设置滤波时采用的体素大小 5cm*5cm*5cm
    grid.setInputCloud(cloud_src);
    grid.filter(*src);

    grid.setInputCloud(cloud_tgt);
    grid.filter(*tgt);
  }
  else
  {
    src = cloud_src;
    tgt = cloud_tgt;
  }

  // 计算表面的法向量和曲率
  PointCloudWithNormals::Ptr points_with_normals_src(new PointCloudWithNormals);
  PointCloudWithNormals::Ptr points_with_normals_tgt(new PointCloudWithNormals);

  pcl::NormalEstimation<PointT, PointNormalT> norm_est; //点云法线估计对象
      // 添加搜索算法 kdtree search  最近的几个点 估计平面 协方差矩阵PCA分解 求解法线
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  norm_est.setSearchMethod(tree);
  norm_est.setKSearch(30); // 指定临近点数量

  norm_est.setInputCloud(src);                //全部的点云
  norm_est.compute(*points_with_normals_src); // 计算表面法线特征
  pcl::copyPointCloud(*src, *points_with_normals_src);
  // 不同类型的点云之间进行类型转换 pcl::copyPointClou

  norm_est.setInputCloud(tgt);
  norm_est.compute(*points_with_normals_tgt);
  pcl::copyPointCloud(*tgt, *points_with_normals_tgt);

  //
  // Instantiate our custom point representation (defined above) ...
  MyPointRepresentation point_representation; // 新的点表示 x,y,z ＋曲率
  // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues(alpha);

  //
  // 配准
  pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg; // 配准对象
  reg.setTransformationEpsilon(1e-6);                                  ///设置收敛判断条件，越小精度越大，收敛也越慢
  // Set the maximum distance between two correspondences (src<->tgt) to 10cm大于此值的点对不考虑
  // Note: adjust this based on the size of your datasets
  reg.setMaxCorrespondenceDistance(0.1); // 10cm大于此值的点对不考虑
  // 设置点表示
  reg.setPointRepresentation(std::make_shared<const MyPointRepresentation>(point_representation));

  reg.setInputSource(points_with_normals_src); // 设置源点云
  reg.setInputTarget(points_with_normals_tgt); // 设置目标点云
  //
  // Run the same optimization in a loop and visualize the results
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource; // Ti Source to target
  PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
  reg.setMaximumIterations(2); ////设置最大的迭代次数，即每迭代两次就认为收敛，停止内部迭代
  for (int i = 0; i < 30; ++i) ////手动迭代，每手动迭代一次，在配准结果视口对迭代的最新结果进行刷新显示
  {

    // 存储点云以便可视化
    points_with_normals_src = reg_result;

    // Estimate
    reg.setInputSource(points_with_normals_src);
    reg.align(*reg_result);

    //accumulate transformation between each Iteration
    Ti = reg.getFinalTransformation() * Ti; // keep track of and accumulate the transformations

    //if the difference between this transformation and the previous one
    //is smaller than the threshold, refine the process by reducing
    //the maximal correspondence distance
    if (fabs((reg.getLastIncrementalTransformation() - prev).sum()) < reg.getTransformationEpsilon())
      reg.setMaxCorrespondenceDistance(reg.getMaxCorrespondenceDistance() - 0.001);

    prev = reg.getLastIncrementalTransformation(); //　

    
  }

  //
  // Get the transformation from target to source
  targetToSource = Ti.inverse(); //deidao
  //
  // Transform target back in source frame
  pcl::transformPointCloud(*cloud_tgt, *output, targetToSource);

  //add the source to the transformed target
  *output += *cloud_src;

  final_transform = targetToSource; 
}