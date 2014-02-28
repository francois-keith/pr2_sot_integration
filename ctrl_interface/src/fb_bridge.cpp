#include "fb_bridge.hpp"
#include <iostream>
// #include <fstream>
// #include <sstream>

FbBridge::FbBridge()
  : m_nh("")
{
  m_pr2_fb = m_nh.subscribe<sensor_msgs::JointState>("joint_states", 1,
    boost::bind(&FbBridge::feedbackCB,this, _1) );
  m_sot_fb = m_nh.advertise<dynamic_graph_bridge_msgs::Vector>("dynamic_graph/fb", 1);
  
  loadConfig();
}

//TODO: This is for orderdering
//HARDCODED!!!!! PR2 ONLY - REFACTOR ME
void FbBridge::loadConfig()
{
  m_sot_order["bl_caster_rotation_joint"] = 0;
  m_sot_order["bl_caster_l_wheel_joint"] = 1;
  m_sot_order["bl_caster_r_wheel_joint"] = 2;
  m_sot_order["br_caster_rotation_joint"] = 3;
  m_sot_order["br_caster_l_wheel_joint"] = 4;
  m_sot_order["br_caster_r_wheel_joint"] = 5;
  m_sot_order["fl_caster_rotation_joint"] = 6;
  m_sot_order["fl_caster_l_wheel_joint"] = 7;
  m_sot_order["fl_caster_r_wheel_joint"] = 8;
  m_sot_order["fr_caster_rotation_joint"] = 9;
  m_sot_order["fr_caster_l_wheel_joint"] = 10;
  m_sot_order["fr_caster_r_wheel_joint"] = 11;
  m_sot_order["torso_lift_joint"] = 12;
  m_sot_order["head_pan_joint"] = 13;
  m_sot_order["head_tilt_joint"] = 14;
  m_sot_order["l_shoulder_pan_joint"] = 15;
  m_sot_order["l_shoulder_lift_joint"] = 16;
  m_sot_order["l_upper_arm_roll_joint"] = 17;
  m_sot_order["l_elbow_flex_joint"] = 18;
  m_sot_order["l_forearm_roll_joint"] = 19;
  m_sot_order["l_wrist_flex_joint"] = 20;
  m_sot_order["l_wrist_roll_joint"] = 21;
  m_sot_order["l_gripper_l_finger_joint"] = 22;
  m_sot_order["l_gripper_l_finger_tip_joint"] = 23;
  m_sot_order["l_gripper_motor_slider_joint"] = 24;
  m_sot_order["l_gripper_motor_screw_joint"] = 25;
  m_sot_order["l_gripper_r_finger_joint"] = 26;
  m_sot_order["l_gripper_r_finger_tip_joint"] = 27;
  m_sot_order["l_gripper_joint"] = 28;
  m_sot_order["laser_tilt_mount_joint"] = 29;
  m_sot_order["r_shoulder_pan_joint"] = 30;
  m_sot_order["r_shoulder_lift_joint"] = 31;
  m_sot_order["r_upper_arm_roll_joint"] = 32;
  m_sot_order["r_elbow_flex_joint"] = 33;
  m_sot_order["r_forearm_roll_joint"] = 34;
  m_sot_order["r_wrist_flex_joint"] = 35;
  m_sot_order["r_wrist_roll_joint"] = 36;
  m_sot_order["r_gripper_l_finger_joint"] = 37;
  m_sot_order["r_gripper_l_finger_tip_joint"] = 38;
  m_sot_order["r_gripper_motor_slider_joint"] = 39;
  m_sot_order["r_gripper_motor_screw_joint"] = 40;
  m_sot_order["r_gripper_r_finger_joint"] = 41;
  m_sot_order["r_gripper_r_finger_tip_joint"] = 42;
  m_sot_order["r_gripper_joint"] = 43;
  m_sot_order["torso_lift_motor_screw_joint"] = 44;
}

void FbBridge::feedbackCB(const sensor_msgs::JointStateConstPtr& msg)
{
  unsigned datasize = m_sot_order.size();
  m_data.data.resize(datasize);
  m_data.data.assign(datasize,0.0);
  for(unsigned i=0;i<msg->position.size();i++)
  {
    std::map<std::string,int>::iterator it = m_sot_order.find(msg->name[i]);
    if(it!=m_sot_order.end())
    {
      m_data.data[it->second] = msg->position[i];
    }
  }
  
  m_sot_fb.publish(m_data);
}
