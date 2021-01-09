cmake_minimum_required(VERSION 2.8.3)
project(chapter2.1)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
  dynamic_reconfigure
)

find_package(PCL 1.8 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/Camera.cfg
)

###########
## Build ##
###########

include_directories(${PCL_INCLUDE_DIRS}  ${catkin_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS}  ${catkin_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})



add_executable(pcl2range src/pcl_to_range.cpp src/point_cloud_range_image_convertor.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(pcl2range
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
)

add_dependencies(pcl2range ${PROJECT_NAME}_gencfg)  # dynamic reconfigure 新加的依赖