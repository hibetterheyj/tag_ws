
#include "fractal/FractalROS.hpp"
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "ros_package_template");
  ros::NodeHandle nodeHandle("~");

  fractal_ros::FractalROS fractal_ros(nodeHandle);

  ros::spin();
  return 0;
}
