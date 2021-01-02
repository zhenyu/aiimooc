#include "point_cloud_range_image_convertor.h"

#include <pcl/visualization/range_image_visualizer.h>
#include <ros/ros.h>  

int main(int argc, char** argv) {
  ros::init(argc, argv, "pcl_to_range");
  ros::NodeHandle n;
  pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
  PointCloudConvertor convertor(range_image_widget);
  ros::Subscriber sub =
      n.subscribe("/rslidar_points", 1000, &PointCloudConvertor::RecvPointCloudCallBack, &convertor);

  ros::spin();
  return 0;
}