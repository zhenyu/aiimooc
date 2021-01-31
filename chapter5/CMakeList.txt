#编译器版本限制
cmake_minimum_required( VERSION 2.8 )

#工程名
project( chapter5 )

#模式
set( CMAKE_BUILD_TYPE Release )
# 添加c++ 14标准支持
set( CMAKE_CXX_FLAGS "-std=c++14 -O3" )

#找 pcl 并链接 
#注意　common io filters visualization　keypoints 分别为其子模块，
# 使用到子模块的需要添加相应的子模块
find_package( PCL REQUIRED COMPONENT common io registration visualization features filters)
include_directories( ${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions( ${PCL_DEFINITIONS} )

# 使用ICP迭代最近点算法
add_executable (contact main.cpp aligner.cpp pair_icp_aligner.cpp)
target_link_libraries (contact ${PCL_LIBRARIES})
