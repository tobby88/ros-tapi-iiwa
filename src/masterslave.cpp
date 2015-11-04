#include "masterslave/masterslave.h"
#include <ros/ros.h>


#define MM_TO_M 1/1000
#define ORIENTATIONBOOST 1
#define RADTORPM (2*M_PI)/60
#define TRANSMISSION_G 169
#define TRANSMISSION_Q5 8.26/6.79
#define TRANSMISSION_Q6N 8.26/5.92
#define TRANSMISSION_Q6P 8.26/6.66

MasterSlave::MasterSlave(ros::NodeHandle& nodeHandle): nh_(nodeHandle)
{
    std::stringstream device_sstream;
    XmlRpc::XmlRpcValue deviceList;
    if(!strcmp(nh_.getNamespace().c_str(),"/Joy"))
    {
        nh_.getParam("/API/Joy",deviceList);
    }
    else if(!strcmp(nh_.getNamespace().c_str(),"/Spacenav"))
    {
        nh_.getParam("/API/Spacenav",deviceList);
    }
    else
    {
        ROS_ERROR("No ControlDevice found!");
        return;
    }
    //TODO: Auswahl des gewünschten Steuerungsgeräts
    for(XmlRpc::XmlRpcValue::iterator it = deviceList.begin(); it!=deviceList.end();it++)
    {
        device_sstream << it->first << "/Velocity";
        velocitySub = nh_.subscribe(device_sstream.str().c_str(),10,&MasterSlave::velocityCallback, this);
        device_sstream.str(std::string());
        device_sstream << it->first << "/Button";
        buttonSub = nh_.subscribe(device_sstream.str().c_str(),10,&MasterSlave::buttonCallback, this);
        device_sstream.str(std::string());
    }
    Q5StateSub = nh_.subscribe("Q5/joint_states",1,&MasterSlave::Q5StateCallback, this);
    Q6nStateSub = nh_.subscribe("Q6N/joint_states",1,&MasterSlave::Q6nStateCallback, this);
    Q6pStateSub = nh_.subscribe("Q6P/joint_states",1,&MasterSlave::Q6pStateCallback, this);


}

void MasterSlave::buttonCallback(const masterslave::ButtonConstPtr &button)
{

}

void MasterSlave::velocityCallback(const geometry_msgs::TwistStampedConstPtr &velocity)
{

}

void MasterSlave::Q5StateCallback(const faulhaber_driver::stateConstPtr &state)
{

}

void MasterSlave::Q6nStateCallback(const faulhaber_driver::stateConstPtr &state)
{

}

void MasterSlave::Q6pStateCallback(const faulhaber_driver::stateConstPtr &state)
{

}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"MasterSlave");
    ros::NodeHandle MasterSlaveNH(argv[1]);
    MasterSlave MasterSlaveControl(MasterSlaveNH);
    ros::spin();
    return 0;
}
