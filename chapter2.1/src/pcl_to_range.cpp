#include "point_cloud_range_image_convertor.h"

#include <aiimooc_szy/CameraConfig.h> 
#include <dynamic_reconfigure/server.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <ros/ros.h>  

int main(int argc, char** argv) {
  ros::init(argc, argv, "range_image");
  ros::NodeHandle n;
  pcl::visualization::RangeImageVisualizer viewer ("Range image");
  PointCloudConvertor convertor(viewer);
  dynamic_reconfigure::Server<aiimooc_szy::CameraConfig> server;
  dynamic_reconfigure::Server<aiimooc_szy::CameraConfig>::CallbackType callback = boost::bind(&PointCloudConvertor::OnDynamicConfigChange, &convertor,_1,_2);  
  server.setCallback(callback);

  ros::Subscriber sub =
      n.subscribe("/rslidar_points", 1000, &PointCloudConvertor::RecvPointCloudCallBack, &convertor);

  ros::spin();
  return 0;
}
