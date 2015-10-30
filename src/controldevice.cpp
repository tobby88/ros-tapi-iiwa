#include "masterslave/controldevice.h"
#include <ros/console.h>
#include <ros/ros.h>
#include <string>
#include <algorithm>
#include <sstream>

ControlDevice::ControlDevice(ros::NodeHandle &nh): nh_(nh)
{
    std::stringstream namespace_sstr;
    curDeviceNum=0;
    rotGain = 1.0;
    transGain = 1.0;
    nh_.getParam("rotGain",rotGain);
    nh_.getParam("transGain",transGain);
    registration();
    curDeviceType = nh_.getNamespace();

    ROS_DEBUG("Das ist der Namespace: %s",curDeviceType.c_str());
    // Test if it's a Gamepad or a Spacenav

    if(strcmp(curDeviceType.c_str(),"/Joy")==0)
    {
        deviceSub = nh_.subscribe<sensor_msgs::Joy>("/joy",10,&ControlDevice::controlDeviceCallback, this);
        ROS_DEBUG("Gamepad");
    }
    else if(strcmp(curDeviceType.c_str(),"/Spacenav")==0)
    {
        deviceSub = nh_.subscribe<sensor_msgs::Joy>("/spacenav/joy",10,&ControlDevice::controlDeviceCallback, this);
        ROS_DEBUG("Spacenav");
    }

    //Create Subnamespace for each Topic of the same ControlDeviceType
    namespace_sstr << "node" << curDeviceNum << "/Velocity";
    axisPub = nh_.advertise<geometry_msgs::TwistStamped>(namespace_sstr.str().c_str(),1);
    namespace_sstr.str(std::string());

    namespace_sstr << "node" << curDeviceNum << "/Buttons";
    buttonsPub = nh_.advertise<masterslave::Button>(namespace_sstr.str().c_str(),1);

    // Check if there are some buttons
    buttonCheck();
}

//Registration of the ControlDevice at the API
void ControlDevice::registration()
{
    std::stringstream param_sstr;
    std::string param_str = nh_.getNamespace();
    do
    {
        param_sstr.str(std::string());
        curDeviceNum ++;
        param_sstr << "/API" << param_str << "/node" << curDeviceNum;
    }
    while(nh_.hasParam(param_sstr.str().c_str()));
    nh_.setParam(param_sstr.str().c_str(),"ControlDevice");
}

void ControlDevice::buttonCheck()
{
    std::string val;
    curButton = 0;
    std::stringstream button_sstream;
    button_sstream << "button_" << curButton;
    while(nh_.hasParam(button_sstream.str().c_str()))
    {
        nh_.getParam(button_sstream.str().c_str(),val);
        buttons[curButton] = val;
        ROS_DEBUG_NAMED("ButtonCheck","Button: %s", button_sstream.str().c_str());
        button_sstream.str(std::string());
        curButton ++;
        button_sstream << "button_" << curButton;



    }

}

ControlDevice::~ControlDevice()
{

}

void ControlDevice::controlDeviceCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
    geometry_msgs::TwistStamped output;
    masterslave::Button outputButton;
    output.header.stamp = ros::Time::now();
    output.header.frame_id = joy->header.frame_id;
    //TODO: Achsenbelegung per Launchfile oder GUI
    if(strcmp(curDeviceType.c_str(),"/Spacenav")==0)
    {
        output.twist.linear.y = transGain*joy->axes[0];
        output.twist.linear.x = transGain*joy->axes[1];
        output.twist.linear.z = transGain*joy->axes[2];
        output.twist.angular.y = rotGain*joy->axes[3];
        output.twist.angular.x = rotGain*joy->axes[4];
        output.twist.angular.z = rotGain*joy->axes[5];
    }
    else if(strcmp(curDeviceType.c_str(),"/Joy")==0)
    {
        output.twist.linear.x = transGain*joy->axes[0];
        output.twist.linear.y = transGain*joy->axes[1];
        output.twist.linear.z = transGain*joy->axes[2];
        output.twist.angular.x = rotGain*joy->axes[3];
        output.twist.angular.y = rotGain*joy->axes[4];
        output.twist.angular.z = rotGain*joy->axes[5];

    }
    // iterate through the buttons map and publish every button press with name and value
    for(std::map<int,std::string>::iterator it=buttons.begin(); it!=buttons.end();it++)
    {
       outputButton.header.stamp = ros::Time::now();
       output.header.frame_id = joy->header.frame_id;
       if(joy->buttons[it->first]==NULL)
       {
           ROS_ERROR("Button is not usable");
       }
       outputButton.state = joy->buttons[it->first];
       outputButton.name = it->second;
       buttonsPub.publish(outputButton);
    }
    axisPub.publish(output);

}





int main(int argc, char** argv)
{
    ros::init(argc,argv, "ControlDevice");
    ros::NodeHandle ControlDeviceNH(argv[1]);
    ControlDevice device(ControlDeviceNH);
    ros::spin();
    return 0;
}
