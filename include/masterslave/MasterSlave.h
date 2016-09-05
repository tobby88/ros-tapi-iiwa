#ifndef MASTERSLAVE_H
#define MASTERSLAVE_H

#include <ros/console.h>
#include <ros/ros.h>
#include <sstream>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

// TF Stuff
#include <eigen_conversions/eigen_msg.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

// Dynamic Reconfigure Stuff
#include <dynamic_reconfigure/server.h>
#include <masterslave/MasterSlaveConfig.h>

class MasterSlave
{
public:
  MasterSlave(ros::NodeHandle &, ros::NodeHandle &);
  ~MasterSlave();

private:
  void configurationCallback(masterslave::MasterSlaveConfig &config, uint32_t level);

  void statemachineThread(const ros::TimerEvent &);

  ros::NodeHandle taskNH_;
  ros::NodeHandle nh_;

  ros::Publisher rcmPub;
  ros::ServiceClient stateService;
};

#endif  // MASTERSLAVE_H
