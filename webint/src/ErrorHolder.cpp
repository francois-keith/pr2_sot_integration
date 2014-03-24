#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <robohow_common_msgs/ConstraintConfig.h>
#include <webint/WebifTaskInfo.h>
#include <algorithm>

//Multithread boost
#include <boost/thread/thread.hpp>

struct NormData
{
  double value;
  double max;
//   double tol;
  NormData() : value(0.0), max(0.0) {}
};

class Holder
{
public:
  Holder(const ros::NodeHandle& nh)
  {
    m_sot_tasks = m_nh.subscribe( "/constraint_config", 1, 
                      (boost::function < void(const robohow_common_msgs::ConstraintConfig::ConstPtr&)>) boost::bind( &Holder::configCB, this, _1 ));
    m_webif = m_nh.advertise<webint::WebifTaskInfo>( "/webif_config", 1);
    
    m_thread = boost::thread(
                boost::bind(
                  &Holder::update, this));
  }
  
  void update()
  {
    try{
        ros::Rate loop(5);
        while(ros::ok())
        {
          webint::WebifTaskInfo out_msg;
    
          for(std::map<std::string, ros::Subscriber>::iterator it=m_errors.begin();
              it!=m_errors.end();it++)
          {
            out_msg.task_name.push_back(it->first);
            std::map<std::string,NormData>::iterator valit = m_cache.find(it->first);
            out_msg.error_norm.push_back(valit->second.value);
            double ratio = valit->second.value / valit->second.max;
            if (std::isnan(ratio) )  //HACK: for existing tasks, but info not available
              out_msg.ratio.push_back(0.0);
            else
              out_msg.ratio.push_back(ratio);
            if(ratio > 0.8)
                out_msg.meta_ratio.push_back("NOT OK");
            else if(ratio > 0.1)
                out_msg.meta_ratio.push_back("SOLVING");
            else
                out_msg.meta_ratio.push_back("OK");
          }
          m_webif.publish(out_msg);
          loop.sleep();
        }
    } catch(boost::thread_interrupted const&) { return; }
  }
  
  ~Holder()
  {
    m_thread.interrupt();
    m_thread.join();
  }
  
private:
  void cachingCB(const std::string& name, const std_msgs::Float64::ConstPtr& msg)
  {
    std::map<std::string,NormData>::iterator valit = m_cache.find(name);
    if(valit!=m_cache.end())
    {
      valit->second.value = msg->data;
      if(valit->second.max == 0.0)
      {
        valit->second.max = msg->data;
      }
    }
  }
  
  void configCB(const robohow_common_msgs::ConstraintConfig::ConstPtr& msg)
  {
    std::map<std::string,bool> tmp;
    
    for(unsigned int i=0;i<msg->constraints.size();i++)
    {
      std::string tmp_taskname = msg->constraints[i].name;
      std::replace( tmp_taskname.begin(), tmp_taskname.end(), '-', '_');
      
      std::map<std::string, ros::Subscriber>::iterator it = m_errors.find(tmp_taskname);
      if(it==m_errors.end())
      { 
        m_errors.insert(std::make_pair(tmp_taskname, m_nh.subscribe( "/sot/"+tmp_taskname+"_error_norm", 1, 
                      (boost::function < void(const std_msgs::Float64::ConstPtr&)>) boost::bind( &Holder::cachingCB, this, tmp_taskname, _1 )) ) );
        m_cache.insert(std::make_pair(tmp_taskname, NormData()));
      }
      tmp.insert(std::make_pair(tmp_taskname,true));
    }
    
    for(std::map<std::string, ros::Subscriber>::iterator it=m_errors.begin();
          it!=m_errors.end();it++)
    {
      std::map<std::string,bool>::iterator sit=tmp.find(it->first);
      if(sit==tmp.end())
      {
        m_errors.erase(it->first);
        m_cache.erase(it->first);
      }
    }

  }
 
  ros::NodeHandle m_nh;
  ros::Subscriber m_sot_tasks;
  ros::Publisher  m_webif;
  std::map<std::string, ros::Subscriber> m_errors;
  std::map<std::string, NormData> m_cache;
  boost::thread m_thread;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv,"errorHolder");
  ros::NodeHandle nh;
  Holder webh(nh);

  ros::MultiThreadedSpinner spinner(5);
  spinner.spin();
  ros::shutdown();
  return 0;
}