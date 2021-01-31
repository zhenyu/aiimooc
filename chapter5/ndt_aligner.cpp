#include <pcl/registration/ndt.h>               //NDT(正态分布)配准类头文件
#include <pcl/filters/approximate_voxel_grid.h> //滤波类头文件

#include "ndt_aligner.h"

void NDTAligner::align(const PointCloud::Ptr cloud_src,
                       const PointCloud::Ptr cloud_tgt,
                       PointCloud::Ptr output,
                       Eigen::Matrix4f &final_transform, bool downsample)
{
  //后续配准是完成对源点云到目标点云的参考坐标系的变换矩阵的估计，
  //得到第二组点云变换到第一组点云坐标系下的变换矩阵
  // 将输入的扫描点云数据过滤到原始尺寸的10%以提高匹配的速度，
  //只对源点云进行滤波，减少其数据量，而目标点云不需要滤波处理
  //因为在NDT算法中在目标点云 对应的 体素网格数据结构的 统计计算不使用单个点，
  //而是使用包含在每个体素单元格中的点的统计数据
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
  approximate_voxel_filter.setInputCloud(cloud_src); // 第二次扫描点云数据作为源点云
  approximate_voxel_filter.filter(*filtered_cloud);
  std::cout << "Filtered cloud contains " << filtered_cloud->size()
            << " data points from room_scan2.pcd" << std::endl;

  // 初始化正态分布(NDT)对象
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

  // 根据输入数据的尺度设置NDT相关参数
  ndt.setTransformationEpsilon(0.01); // 为终止条件设置最小转换差异
  ndt.setStepSize(0.1);               // 为more-thuente线搜索设置最大步长
  ndt.setResolution(1.0);             // 设置NDT网格网格结构的分辨率（voxelgridcovariance）

  //以上参数在使用房间尺寸比例下运算比较好，但是如果需要处理例如一个咖啡杯子的扫描之类更小的物体，需要对参数进行很大程度的缩小

  //设置匹配迭代的最大次数，这个参数控制程序运行的最大迭代次数，一般来说这个限制值之前优化程序会在epsilon变换阀值下终止
  //添加最大迭代次数限制能够增加程序的鲁棒性阻止了它在错误的方向上运行时间过长
  ndt.setMaximumIterations(35);

  ndt.setInputSource(filtered_cloud); //源点云
  // Setting point cloud to be aligned to.
  ndt.setInputTarget(cloud_tgt); //目标点云

  // 设置使用机器人测距法得到的粗略初始变换矩阵结果
  Eigen::AngleAxisf init_rotation(0.6931, Eigen::Vector3f::UnitZ());
  Eigen::Translation3f init_translation(1.79387, 0.720047, 0);
  Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();

  // 计算需要的刚体变换以便将输入的源点云匹配到目标点云
  ndt.align(*output, init_guess);
  //这个地方的output_cloud不能作为最终的源点云变换，因为上面对点云进行了滤波处理
  std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged()
            << " score: " << ndt.getFitnessScore() << std::endl;

  // 使用创建的变换对为过滤的输入点云进行变换
  final_transform = ndt.getFinalTransformation();
                        pcl::transformPointCloud(*cloud_src, *output, final_transform);
  //add the source to the transformed target
  *output += *cloud_src;
}