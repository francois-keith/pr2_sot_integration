#include "fb_bridge.hpp"
#include <iostream>
// #include <fstream>
// #include <sstream>

FbBridge::FbBridge()
  : m_nh("")
{
  m_pr2_fb = m_nh.subscribe<sensor_msgs::JointState>("joint_states", 1,
    boost::bind(&FbBridge::feedbackCB,this, _1) );
  m_sot_fb = m_nh.advertise<dynamic_graph_bridge::Vector>("dynamic_graph/fb", 1);
}

void FbBridge::feedbackCB(const sensor_msgs::JointStateConstPtr& msg)
{
  m_data.data.resize(msg->velocity.size());
  for(unsigned i=0;i<m_data.data.size();i++)
  {
    m_data.data[i] = msg->velocity[i];
  }
  m_sot_fb.publish(m_data);
}