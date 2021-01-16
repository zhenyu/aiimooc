cmake_minimum_required(VERSION 3.0)
project (rotation)
 
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
 
add_executable (rotation transform_szy.cpp)
target_link_libraries (rotation Eigen3::Eigen)
