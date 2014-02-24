#include <ros/ros.h>
#include "fb_bridge.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fb_bridge");
  FbBridge fbbridge;
  
  ros::spin();
  return 0;
}