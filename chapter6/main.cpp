#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

// Types
typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ColorHandlerT;
void computeNorm(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_ptr, pcl::PointCloud<pcl::Normal>::Ptr &normals_ptr)
{
  // =====【2】计算法线========创建法线估计类====================================
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud_ptr);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  ne.setSearchMethod(tree); //设置近邻搜索算法
  // 输出点云 带有法线描述
  pcl::PointCloud<pcl::Normal> &normals = *normals_ptr;
  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch(0.03); //半价内搜索临近点 3cm
  // 计算表面法线特征
  ne.compute(normals);
}
void computeFeature(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_ptr, pcl::PointCloud<pcl::Normal>::Ptr &normals_ptr, pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fe_ptr)
{
  pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
  fpfh.setInputCloud(cloud_ptr);
  fpfh.setInputNormals(normals_ptr);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>());
  fpfh.setSearchMethod(tree2); //设置近邻搜索算法
  //注意：此处使用的半径必须要大于估计表面法线时使用的半径!!!
  fpfh.setRadiusSearch(0.05);
  //计算pfh特征值
  fpfh.compute(*fe_ptr);

  /*
  fest.setRadiusSearch(5);
  fest.setInputCloud(scene);
  fest.setInputNormals(scene);
  fest.compute(*scene_features);
  */
}
void downSample(const float leaf, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_ptr)
{
  // Downsample
  pcl::VoxelGrid<pcl::PointXYZ> grid;
  grid.setLeafSize(leaf, leaf, leaf);
  grid.setInputCloud(cloud_ptr);
  grid.filter(*cloud_ptr);
}
// Align a rigid object to a scene with clutter and occlusions
int main(int argc, char **argv)
{
  // Point clouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr object(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr object_aligned(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr scene(new pcl::PointCloud<pcl::PointXYZ>);

  // Get input object and scene
  if (argc != 3)
  {
    pcl::console::print_error("Syntax is: %s object.pcd scene.pcd\n", argv[0]);
    return (1);
  }

  // Load object and scene
  pcl::console::print_highlight("Loading point clouds...\n");
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *object) < 0 ||
      pcl::io::loadPCDFile<pcl::PointXYZ>(argv[2], *scene) < 0)
  {
    pcl::console::print_error("Error loading object/scene file!\n");
    return (1);
  }

  cout << "object\n"
       << pcl::getFieldsList(*object) << endl;
  cout << "scene\n"
       << pcl::getFieldsList(*scene) << endl;

  pcl::console::print_highlight("Downsampling objec...\n");
  downSample(0.1f, object);
  pcl::console::print_highlight("Downsampling scene...\n");
  downSample(3, scene);

  cout << "scene: " << scene->points.size() << ", " << object->points.size() << endl;

  // Estimate normals
  pcl::console::print_highlight("Estimating scene normals...\n");
  pcl::PointCloud<pcl::Normal>::Ptr scene_normals_ptr(new pcl::PointCloud<pcl::Normal>);
  computeNorm(scene, scene_normals_ptr);

  pcl::console::print_highlight("Estimating object normals...\n");
  pcl::PointCloud<pcl::Normal>::Ptr object_normals_ptr(new pcl::PointCloud<pcl::Normal>);
  computeNorm(object, object_normals_ptr);

  // Estimate features
  pcl::console::print_highlight("Estimating  scene features...\n");
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr scene_fe_ptr(new pcl::PointCloud<pcl::FPFHSignature33>()); //fphf特征
  computeFeature(scene, scene_normals_ptr, scene_fe_ptr);

  pcl::console::print_highlight("Estimating  object features...\n");
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr object_fe_ptr(new pcl::PointCloud<pcl::FPFHSignature33>()); //fphf特征
  computeFeature(object, object_normals_ptr, object_fe_ptr);

  // Perform alignment
  pcl::console::print_highlight("Starting alignment...\n");
  pcl::SampleConsensusPrerejective<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> align;
  align.setInputSource(object);
  align.setSourceFeatures(object_fe_ptr);
  align.setInputTarget(scene);
  align.setTargetFeatures(scene_fe_ptr);
  align.setMaximumIterations(50000);               // Number of RANSAC iterations
  align.setNumberOfSamples(3);                     // Number of points to sample for generating/prerejecting a pose
  align.setCorrespondenceRandomness(5);            // Number of nearest features to use
  align.setSimilarityThreshold(0.9f);              // Polygonal edge length similarity threshold
  align.setMaxCorrespondenceDistance(2.5f * leaf); // Inlier threshold
  align.setInlierFraction(0.25f);                  // Required inlier fraction for accepting a pose hypothesis
  {
    pcl::ScopeTime t("Alignment");
    align.align(*object_aligned);
  }

  if (align.hasConverged())
  {
    // Print results
    printf("\n");
    Eigen::Matrix4f transformation = align.getFinalTransformation();
    pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(0, 0), transformation(0, 1), transformation(0, 2));
    pcl::console::print_info("R = | %6.3f %6.3f %6.3f | \n", transformation(1, 0), transformation(1, 1), transformation(1, 2));
    pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(2, 0), transformation(2, 1), transformation(2, 2));
    pcl::console::print_info("\n");
    pcl::console::print_info("t = < %0.3f, %0.3f, %0.3f >\n", transformation(0, 3), transformation(1, 3), transformation(2, 3));
    pcl::console::print_info("\n");
    pcl::console::print_info("Inliers: %i/%i\n", align.getInliers().size(), object->size());

    // Show alignment
    pcl::visualization::PCLVisualizer visu("Alignment");
    visu.addPointCloud(scene, ColorHandlerT(scene, 0.0, 255.0, 0.0), "scene");
    visu.addPointCloud(object_aligned, ColorHandlerT(object_aligned, 0.0, 0.0, 255.0), "object_aligned");
    visu.addPointCloud(object, ColorHandlerT(object, 255, 0.0, 0), "object");

    visu.spin();
  }
  else
  {
    pcl::console::print_error("Alignment failed!\n");
    return (1);
  }

  return (0);
}
