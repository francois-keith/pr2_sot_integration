#include "vel_bridge.hpp"
#include <iostream>
#include <fstream>
#include <sstream>

#include <urdf_parser/urdf_parser.h>
VBridge::VBridge()
  : m_nh("")
{
  m_vel_sub = m_nh.subscribe<sensor_msgs::JointState>("sot_cmdvel", 1,
    boost::bind(&VBridge::setpointCB,this, _1) );
  
  loadConfig();
  createStreams();
}

void VBridge::loadConfig()
{
  m_jntnames.clear();
  m_jntnames.push_back("r_shoulder_pan");
  m_jntnames.push_back("r_shoulder_lift");
  m_jntnames.push_back("r_upper_arm_roll");
  m_jntnames.push_back("r_elbow_flex");
  m_jntnames.push_back("r_forearm_roll");
  m_jntnames.push_back("r_wrist_flex");
  m_jntnames.push_back("r_wrist_roll");
  
  m_jntnames.push_back("l_shoulder_pan");
  m_jntnames.push_back("l_shoulder_lift");
  m_jntnames.push_back("l_upper_arm_roll");
  m_jntnames.push_back("l_elbow_flex");
  m_jntnames.push_back("l_forearm_roll");
  m_jntnames.push_back("l_wrist_flex");
  m_jntnames.push_back("l_wrist_roll");
  
  m_jntnames.push_back("torso_lift");
}

void VBridge::createStreams()
{
  m_vel_pubs.resize(m_jntnames.size());
  m_cmdvel.resize(m_jntnames.size());
  for(unsigned int i=0;i<m_jntnames.size();i++)
  {
    std::stringstream ss;
    ss << m_jntnames[i] + "_velocity_controller/command";
    m_vel_pubs[i] = m_nh.advertise<std_msgs::Float64>(ss.str(), 1);
    m_sot2jnt.insert(std::make_pair(m_jntnames[i]+"_joint",i));
  }
}

bool VBridge::loadConfig(const std::string& urdf_name)
{
  //read from file first
  std::string xml_string;
  std::fstream xml_file(urdf_name.c_str(), std::fstream::in);
  while ( xml_file.good() ) {
    std::string line;
    std::getline( xml_file, line);
    xml_string += (line + "\n");
  }
  xml_file.close();
  
  //parse string
  boost::shared_ptr<urdf::ModelInterface> robot = urdf::parseURDF(xml_string);
  if(!robot)
    return false;
  
  for(std::map<std::string, boost::shared_ptr<urdf::Joint> >::iterator it=robot->joints_.begin();
      it!=robot->joints_.end();it++)
   {
     if(it->second->type==urdf::Joint::REVOLUTE || it->second->type==urdf::Joint::CONTINUOUS ||
       it->second->type==urdf::Joint::PRISMATIC)
          m_jntnames.push_back(it->first);
   }
   
  for(unsigned int i=0;i<m_jntnames.size();i++)
    std::cout << m_jntnames[i] << std::endl;
   
  return true;
}

void VBridge::streamOut()
{
  for(unsigned int i=0;i<m_vel_pubs.size();i++)
    m_vel_pubs[i].publish(m_cmdvel[i]);
}

void VBridge::setpointCB(const sensor_msgs::JointStateConstPtr& msg)
{
  //To ensure safety commands, set all to zero first
  for(unsigned int i=0;i<m_cmdvel.size();i++)
    m_cmdvel[i].data = 0.0; 
  
  //First, ensure that the velocity is not empty, if so, raise an error
  if(msg->velocity.size() < 1)
  {
    ROS_ERROR("VELOCITY BRIDGE: I received an empty velocity message - command refused");
    streamOut(); //Zeros
    return;
  }
  if(msg->velocity.size() != msg->name.size() )
  {
    ROS_ERROR("VELOCITY BRIDGE: velocity message has different size of name size - command refused");
    streamOut(); //Zeros
    return;
  }
  
  for(unsigned int i=0;i<msg->name.size();i++)
  {
    std::map<std::string,int>::iterator it=m_sot2jnt.find(msg->name[i]);
    if(it!=m_sot2jnt.end())
      m_cmdvel[it->second].data = msg->velocity[i];
  }
  
  //stream out
    streamOut();
}