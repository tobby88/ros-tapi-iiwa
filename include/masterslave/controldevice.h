#ifndef CONTROLDEVICE_H
#define CONTROLDEVICE_H

#include <ros/ros.h>

#include "geometry_msgs/TwistStamped.h"
#include "masterslave/Button.h"
#include "sensor_msgs/Joy.h"
#include <string>
#include <vector>
#include <map>
#include <string>
#include <algorithm>
#include <sstream>
#include <dynamic_reconfigure/server.h>
#include <masterslave/controldeviceConfig.h>


class ControlDevice {
public:
    ControlDevice(ros::NodeHandle& globalNH, ros::NodeHandle& nh);
    void configurationCallback(masterslave::controldeviceConfig &config, uint32_t level);
    ~ControlDevice();


private:
    void controlDeviceCallback(const sensor_msgs::Joy::ConstPtr& joy);
    void registration();
    void buttonCheck();

    bool errorShown;
    ros::NodeHandle globalNH_;
    ros::NodeHandle nh_;
    ros::Subscriber deviceSub;
    ros::Publisher  axisPub;
    ros::Publisher  buttonsPub;
    std::map<int, std::string> buttons;
    double rotGain;
    double transGain;
    double joyThresh;
    std::string curDeviceType;
    std::string apiDevice;
    int curDeviceNum;
    sensor_msgs::Joy joy_old;



};


#endif // CONTROLDEVICE_H
