#编译器版本限制
cmake_minimum_required( VERSION 2.8 )

#工程名
project( chapter4 )

#模式
set( CMAKE_BUILD_TYPE Release )

set( CMAKE_CXX_FLAGS "-std=c++14 -O3" )

#找 pcl 并链接 
#注意　common io filters visualization　keypoints 分别为其子模块，
# 使用到子模块的需要添加相应的子模块
find_package( PCL REQUIRED COMPONENT io common visualization filters segmentation)
include_directories( ${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions( ${PCL_DEFINITIONS} )


# 4 欧式聚类分割　平面模型分割平面　平面上按　聚类得到　多个点云团
add_executable (seg_cluster seg_cluster.cpp)
target_link_libraries (seg_cluster ${PCL_LIBRARIES})

