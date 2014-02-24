//Simple converter datatype from JointState to Vector (containts velocities) for SoT
//Enea Scioni
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <dynamic_graph_bridge/Vector.h>

#include <string>

class FbBridge
{
  public:
    FbBridge();
    void feedbackCB(const sensor_msgs::JointStateConstPtr& msg);
    
  private:
    ros::Publisher m_sot_fb;
    ros::Subscriber m_pr2_fb;
    ros::NodeHandle m_nh;
    
    dynamic_graph_bridge::Vector m_data;
};
