

#include <ros/ros.h>
#include <hector_barrel_detection/hector_barrel_detection.h>




int main(int argc, char **argv)
{
  ros::init(argc, argv, "hector_barrel_detection");

  ROS_INFO("Starting Head Control Node");
  barrel_detection::BarrelDetection barrel_detector;
  ros::spin();
  exit(0);
}
