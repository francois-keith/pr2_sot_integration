#include <ros/ros.h>
#include <robohow_common_msgs/ConstraintConfig.h>

int main(int argc, char** argv)
{
  ros::init(argc,argv,"task_sender");
  ros::NodeHandle nh;
  ros::Publisher pub;
  
  pub = nh.advertise<robohow_common_msgs::ConstraintConfig>("/constraint_config",1);
  
  robohow_common_msgs::ConstraintConfig msg;
  msg.constraints.resize(5);
  msg.constraints[0].name = "task_1";
  msg.constraints[1].name = "task_xy";
  msg.constraints[2].name = "task_angle";
  msg.constraints[3].name = "task_elbow";
  msg.constraints[4].name = "task_joint_limtis";
  ros::Rate loop(1);
  
  while(ros::ok())
  {
    pub.publish(msg);
    
    loop.sleep();
  }

  return 0;
}