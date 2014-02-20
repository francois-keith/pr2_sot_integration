//test only, to be removed later on
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

int main(int argc, char** argv)
{
  ros::init(argc,argv,"fake_sot");
  ros::NodeHandle nh;
  
  ros::Publisher cmdvel = nh.advertise<sensor_msgs::JointState>("/sot_cmdvel", 1);
  ros::Rate loop_rate(10);
  sensor_msgs::JointState velmsg;
  velmsg.name.resize(2);
  velmsg.position.resize(2);
  velmsg.velocity.resize(2);
  velmsg.effort.resize(2);
  velmsg.name[0] = "r_wrist_roll_joint";
  velmsg.name[1] = "l_wrist_roll_joint";
  velmsg.velocity[0] = 0.2;
  velmsg.velocity[1] = 0.1;
  while (ros::ok())
  {
    cmdvel.publish(velmsg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}