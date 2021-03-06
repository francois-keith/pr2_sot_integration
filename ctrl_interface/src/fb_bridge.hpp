//Simple converter datatype from JointState to Vector (containts velocities) for SoT
//Enea Scioni
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <dynamic_graph_bridge_msgs/Vector.h>
#include <map>
#include <string>

class FbBridge
{
  public:
    FbBridge();
    void feedbackCB(const sensor_msgs::JointStateConstPtr& msg);
    void loadConfig();
    
  private:
    ros::Publisher m_sot_fb;
    ros::Subscriber m_pr2_fb;
    ros::NodeHandle m_nh;
    
    dynamic_graph_bridge_msgs::Vector m_data;
    std::map<std::string,int> m_sot_order;
};
