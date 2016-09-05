#include "masterslave/MasterSlave.h"
#include <ros/ros.h>

#define MM_TO_M 1 / 1000
#define DEG_TO_RAD M_PI / 180

MasterSlave::MasterSlave(ros::NodeHandle& controlDeviceNH, ros::NodeHandle& taskNodeHandle)
  : nh_(controlDeviceNH), taskNH_(taskNodeHandle)
{
  dynamic_reconfigure::Server<masterslave::MasterSlaveConfig> server(taskNH_);
  dynamic_reconfigure::Server<masterslave::MasterSlaveConfig>::CallbackType f;

  f = boost::bind(&MasterSlave::configurationCallback, this, _1, _2);

  server.setCallback(f);

  // Winkel werden in Rad/s übergeben (siehe server.cpp)

  ros::spin();
}

MasterSlave::~MasterSlave()
{
}

void MasterSlave::configurationCallback(masterslave::MasterSlaveConfig& config, uint32_t level)
{
  newState = static_cast<OPENIGTL_STATE>(config.cur_state);
  rosRate = config.rosRate;
  ROS_INFO_STREAM("current State: " << curState << " new State: " << newState);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "MasterSlave");

  ros::NodeHandle nodeHandle("dynamicReconfigure");
  ros::NodeHandle ControlDeviceNH(argv[1]);

  MasterSlave MasterSlaveControl(ControlDeviceNH, nodeHandle);

  return 0;
}
