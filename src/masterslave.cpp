#include "masterslave/masterslave.h"
#include <ros/ros.h>


#define MM_TO_M 1/1000
#define DEG_TO_RAD M_PI/180

MasterSlave::MasterSlave(ros::NodeHandle& controlDeviceNH): nh_(controlDeviceNH)
{
    start_ = false;
    apertureLimit = 45;
    heightSafety = 0.05;
    rosRate = 50;

    dynamic_reconfigure::Server<masterslave::masterslaveConfig> server;
    dynamic_reconfigure::Server<masterslave::masterslaveConfig>::CallbackType f;

    f = boost::bind(&MasterSlave::configurationCallback,this,_1,_2);

    server.setCallback(f);

    // Winkel werden in Rad/s übergeben (siehe server.cpp)
    statePub = nh_.advertise<std_msgs::UInt8>("/openIGTLState",1);
    task = new LaparoscopeTask(nh_,rosRate);

}

void MasterSlave::configurationCallback(masterslave::masterslaveConfig &config, uint32_t level)
{
    rosRate = config.rosRate;
    start_ = config.start;
    ROS_WARN_STREAM("start: " << start_);
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"MasterSlave");

    ros::NodeHandle ControlDeviceNH(argv[1]);
    MasterSlave MasterSlaveControl(ControlDeviceNH);

    return 0;
}
