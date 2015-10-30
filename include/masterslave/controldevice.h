#ifndef CONTROLDEVICE_H
#define CONTROLDEVICE_H

#include <ros/ros.h>

#include "geometry_msgs/TwistStamped.h"
#include "masterslave/Button.h"
#include "sensor_msgs/Joy.h"
#include <string>
#include <vector>

class ControlDevice {
public:
    ControlDevice(ros::NodeHandle& nh);
    ~ControlDevice();


private:
    void controlDeviceCallback(const sensor_msgs::Joy::ConstPtr& joy);
    void registration();
    void buttonCheck();
    ros::NodeHandle nh_;
    ros::Subscriber deviceSub;
    ros::Publisher  axisPub;
    ros::Publisher  buttonsPub;
    std::map<int, std::string> buttons;
    double rotGain;
    double transGain;
    std::string curDeviceType;
    int curDeviceNum;
    int curButton;



};


#endif // CONTROLDEVICE_H
