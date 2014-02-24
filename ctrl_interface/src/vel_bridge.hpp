#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

#include <string>

class VBridge
{
  public:
    VBridge();
    bool loadConfig(const std::string& urdf_file);
    void loadConfig();
    void createStreams();
    void setpointCB(const sensor_msgs::JointStateConstPtr& msg);
    void streamOut();
    
  private:
    std::vector<ros::Publisher> m_vel_pubs;
    ros::Subscriber m_vel_sub;
    ros::NodeHandle m_nh;
    std::vector<std_msgs::Float64> m_cmdvel;
    std::map<std::string,int> m_sot2jnt;
    std::vector<std::string> m_jntnames;
};
