#include "masterslave/ControlDevice.h"
#include <ros/console.h>
#include <ros/ros.h>


ControlDevice::ControlDevice(ros::NodeHandle &nh): nh_(nh)
{
    std::stringstream namespace_sstr;
    errorShown = false;
    curDeviceNum=0;

    rotGain = 0.002;
    transGain = 0.001;

    dynamic_reconfigure::Server<masterslave::ControlDeviceConfig> server;
    dynamic_reconfigure::Server<masterslave::ControlDeviceConfig>::CallbackType f;
    f = boost::bind(&ControlDevice::configurationCallback,this ,_1,_2);
    server.setCallback(f);


    registration();
    curDeviceType = nh_.getNamespace();

    // Check if it's a Gamepad or a Spacenav
    if(strcmp(curDeviceType.c_str(),"/Joy")==0)
    {
        deviceSub = nh_.subscribe<sensor_msgs::Joy>("/joy",1,&ControlDevice::controlDeviceCallback, this);
        ROS_DEBUG("Gamepad");
    }
    else if(strcmp(curDeviceType.c_str(),"/Spacenav")==0)
    {
        deviceSub = nh_.subscribe<sensor_msgs::Joy>("/spacenav/joy",1,&ControlDevice::controlDeviceCallback, this);
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
    ros::spin();
}

ControlDevice::~ControlDevice()
{
    nh_.deleteParam(apiDevice.c_str());
    nh_.shutdown();
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
    apiDevice = param_sstr.str();
}

void ControlDevice::buttonCheck()
{
    std::stringstream button_sstream;
    button_sstream << "button";
    if(nh_.hasParam(button_sstream.str().c_str()))
    {
        XmlRpc::XmlRpcValue buttonList;
        nh_.getParam(button_sstream.str().c_str(),buttonList);
        for(XmlRpc::XmlRpcValue::iterator it=buttonList.begin();it!=buttonList.end();it++)
        {
            buttons[std::atoi(it->first.c_str())] = (std::string&)(it->second);
            ROS_WARN_NAMED("ButtonCheck","found: Functions %s (Button: %s)", ((std::string&)(it->second)).c_str(), it->first.c_str());
        }

    }

}

void ControlDevice::configurationCallback(masterslave::ControlDeviceConfig &config, uint32_t level)
{
    rotGain = config.rotGain;
    transGain = config.transGain;
    joyThresh = config.joyThresh;
    ROS_WARN_STREAM_NAMED("Dynamic Reconfigure","rotGain: " << rotGain << " transGain: " << transGain << " joyThresh: " << joyThresh);
}

void ControlDevice::controlDeviceCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
    if(joy_old.buttons.empty())
    {
        joy_old = *joy;
    }
    geometry_msgs::TwistStamped output;
    sensor_msgs::Joy filteredInput;
    masterslave::Button outputButton;
    output.header.stamp = ros::Time::now();
    output.header.frame_id = joy->header.frame_id;

    for(int i=0;i<6;i++)
    {
        filteredInput.axes.push_back(joy->axes[i]);
        if(fabs(joy->axes[i]) < joyThresh){
            filteredInput.axes[i] = 0.0;
        }
    }
    if(strcmp(curDeviceType.c_str(),"/Spacenav")==0)
    {
        output.twist.linear.y = transGain*filteredInput.axes[1];
        output.twist.linear.x = transGain*filteredInput.axes[0];
        output.twist.linear.z = transGain*filteredInput.axes[2];
        output.twist.angular.y = rotGain*filteredInput.axes[4];
        output.twist.angular.x = rotGain*filteredInput.axes[3];
        output.twist.angular.z = rotGain*filteredInput.axes[5];
    }
    else if(strcmp(curDeviceType.c_str(),"/Joy")==0)
    {
        output.twist.linear.x = transGain*filteredInput.axes[0];
        output.twist.linear.y = transGain*filteredInput.axes[1];
        //output.twist.linear.z = transGain*joy->axes[2];
        output.twist.angular.x = rotGain*filteredInput.axes[3];
        output.twist.angular.y = rotGain*filteredInput.axes[4];
        //output.twist.angular.z = rotGain*joy->axes[5];

    }
    // iterate through the buttons map and publish every button press with name and value
    for(std::map<int,std::string>::iterator it=buttons.begin(); it!=buttons.end();it++)
    {

       outputButton.header.stamp = ros::Time::now();
       outputButton.header.frame_id = joy->header.frame_id;
       // Idee: unerreichbare Buttons nicht senden?!
       ROS_WARN_COND(joy->buttons.capacity()<=it->first&& !errorShown,"Button %d is not accessible",it->first);
       if(joy->buttons.capacity()<=it->first)
       {
           outputButton.state = -1;
       }
       else
       {
          outputButton.state = joy->buttons[it->first];
       }
       outputButton.name = it->second;
       if(joy->buttons[it->first]!=joy_old.buttons[it->first])
       {
            buttonsPub.publish(outputButton);
       }

    }
    errorShown = true;

    axisPub.publish(output);
    joy_old = *joy;

}





int main(int argc, char** argv)
{
    ros::init(argc,argv, "ControlDevice");
    ros::NodeHandle ControlDeviceNH(argv[1]);

    ControlDevice device(ControlDeviceNH);
    return 0;
}
