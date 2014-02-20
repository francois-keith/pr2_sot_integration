//NOTE Naming convention
// Joint names from URDF file (PR2)
// JNT_BASE_NAME + _joint -> joint name
// JNT_BASE_NAME + _velocity_controller -> controllers on pr2_controller_manager 
// JNT_BASE_NAME + _velocity_controller/command -> command topic

#include <ros/ros.h>
#include "vel_bridge.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vel_bridge");
  VBridge vbridge;
  
  ros::spin();
  return 0;
}