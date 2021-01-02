#include "point_cloud_range_image_convertor.h"

#include <ros/ros.h>  

int main(int argc, char** argv) {
  ros::init(argc, argv, "pcl_to_range");
  ros::NodeHandle n;
  PointCloudConvertor convertor;
  ros::Subscriber sub =
      n.subscribe("/rslidar_points", 1000, &PointCloudConvertor::RecvPointCloudCallBack, &convertor);

  ros::spin();
  return 0;
}