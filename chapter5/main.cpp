#include <Eigen/Geometry>
#include <pcl/io/pcd_io.h>                    //PCD文件打开存储类头文件
#include <pcl/registration/transforms.h>      //变换矩阵类头文件

#include "aligner.h"

//申明一个结构体方便对点云以 文件名 和 点云对象进行 成对处理 和 管理点云，
// 处理过程中可以同时接受多个点云文件的输入
struct PCD
{
  PointCloud::Ptr cloud; //点云共享指针
  std::string f_name;    //文件名称

  PCD() : cloud(new PointCloud){};
};

struct PCDComparator //文件比较处理
{
  bool operator()(const PCD &p1, const PCD &p2)
  {
    return (p1.f_name < p2.f_name); //文件名是否相等　
  }
};


////////////////////////////////////////////////////////////////////////////////
/** \brief Load a set of PCD files that we want to register together
  * \param argc the number of arguments (pass from main ())
  * \param argv the actual command line arguments (pass from main ())
  * \param models the resultant vector of point cloud datasets
  */
//加载点云并保存在总体的点云列表中
void loadData(int argc, char **argv, std::vector<PCD, Eigen::aligned_allocator<PCD>> &models)
{
  std::string extension(".pcd");
  // 第一个参数是命令本身，所以要从第二个参数开始解析
  for (int i = 2; i < argc; i++)
  {
    std::string fname = std::string(argv[i]);
    // PCD文件名至少为5个字符大小字符串（因为后缀名.pcd就已经占了四个字符位置）
    if (fname.size() <= extension.size())
      continue;

    std::transform(fname.begin(), fname.end(), fname.begin(), (int (*)(int))tolower);
    //检查参数是否为一个pcd后缀的文件
    if (fname.compare(fname.size() - extension.size(), extension.size(), extension) == 0)
    {
      //加载点云并保存在总体的点云列表中
      PCD m;
      m.f_name = argv[i];                      //文件名称
      pcl::io::loadPCDFile(argv[i], *m.cloud); //点云共享指针
      //从点云中移除NAN点也就是无效点
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*m.cloud, *m.cloud, indices);
      models.push_back(m);
    }
  }
}


int main(int argc, char **argv)
{
  // 存储管理所有打开的点云
  std::vector<PCD, Eigen::aligned_allocator<PCD>> data;
  loadData(argc, argv, data); // 加载所有点云到data

  // 检查输入
  if (data.empty())
  {
    PCL_ERROR("Syntax is: %s <source.pcd> <target.pcd> [*]", argv[0]);
    PCL_ERROR("[*] - multiple files can be added. The registration results of (i, i+1) will be registered against (i+2), etc");
    return (-1);
  }
  PCL_INFO("Loaded %d datasets.", (int)data.size());

  PointCloud::Ptr result(new PointCloud), source, target;
  Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity(), pairTransform;
  
  char * aligner_name = argv[1];
  Aligner* aligner = Aligner::get_aligner(aligner_name);
  for (size_t i = 1; i < data.size(); ++i) //循环处理所有点云
  {
    source = data[i - 1].cloud; // 连续配准
    target = data[i].cloud;     // 相邻两组点云

    //调用子函数完成一组点云的配准，temp返回配准后两组点云在第一组点云坐标下的点云
    PointCloud::Ptr temp(new PointCloud);

    // pairTransform返回从目标点云target到source的变换矩阵
    aligner->align(source, target, temp, pairTransform, true);

    //把当前两两配准后的点云temp转化到全局坐标系下返回result
    pcl::transformPointCloud(*temp, *result, GlobalTransform);
    
    //用当前的两组点云之间的变换更新全局变换
    GlobalTransform = GlobalTransform * pairTransform;

  }
  delete aligner;
  std::stringstream ss;
  ss << "reg_" << aligner_name<< ".pcd";
  pcl::io::savePCDFile(ss.str (), *result, true);
}
