
#include <ros/ros.h> 

#include "eigen_caculator.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "eigenfeature");
  ros::NodeHandle n;
  EigenCaculator caculator;
  ros::Subscriber sub =
      n.subscribe("/rslidar_points", 1000, &EigenCaculator::RecvPointCloudCallBack, &caculator);

  ros::spin();
  return 0;
}
