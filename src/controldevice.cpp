#include "masterslave/controldevice.h"
#include <ros/ros.h>
#include <string>
#include <algorithm>
#include <sstream>

ControlDevice::ControlDevice(ros::NodeHandle &nh): nh_(nh)
{

    std::string param_str = nh_.getNamespace();
    std::stringstream param_sstr;
    std::stringstream namespace_sstr;
    curDeviceNum=0;
    do{
        param_sstr.str(std::string());
        curDeviceNum ++;
        param_sstr << "/API" << param_str << "/node" << curDeviceNum;
    }
    while(nh_.hasParam(param_sstr.str().c_str()));

    nh_.setParam(param_sstr.str().c_str(),"ControlDevice");
    curDeviceType = nh_.getNamespace();
    ROS_INFO("Das ist der Namespace: %s",curDeviceType.c_str());
    if(strcmp(curDeviceType.c_str(),"/Joy")==0)
    {
        deviceSub = nh_.subscribe<sensor_msgs::Joy>("/joy",10,&ControlDevice::controlDeviceCallback, this);
        ROS_INFO("Gamepad");
    }
    else if(strcmp(curDeviceType.c_str(),"/Spacenav")==0)
    {
        deviceSub = nh_.subscribe<sensor_msgs::Joy>("/spacenav/joy",10,&ControlDevice::controlDeviceCallback, this);
        ROS_INFO("Spacenav");
    }
    namespace_sstr << "node" << curDeviceNum << "/Velocity";
    apiSub = nh_.advertise<geometry_msgs::TwistStamped>(namespace_sstr.str().c_str(),1);
}

ControlDevice::~ControlDevice()
{

}

void ControlDevice::controlDeviceCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
    geometry_msgs::TwistStamped output;
    output.header.stamp = ros::Time::now();
    output.header.frame_id = joy->header.frame_id;
    if(strcmp(curDeviceType.c_str(),"/Spacenav")==0)
    {
        output.twist.linear.y = joy->axes[0];
        output.twist.linear.x = joy->axes[1];
        output.twist.linear.z = joy->axes[2];
        output.twist.angular.y = joy->axes[3];
        output.twist.angular.x = joy->axes[4];
        output.twist.angular.z = joy->axes[5];
    }
    else if(strcmp(curDeviceType.c_str(),"/Joy")==0)
    {
        output.twist.linear.x = joy->axes[0];
        output.twist.linear.y = joy->axes[1];
        output.twist.linear.z = joy->axes[2];

        output.twist.angular.x = joy->axes[3];
        output.twist.angular.y = joy->axes[4];
        output.twist.angular.z = joy->axes[5];

    }

    apiSub.publish(output);

}





int main(int argc, char** argv)
{
    ros::init(argc,argv, "ControlDevice");
    ros::NodeHandle ControlDeviceNH(argv[1]);
    ControlDevice device(ControlDeviceNH);
    ros::spin();
    return 0;
}
