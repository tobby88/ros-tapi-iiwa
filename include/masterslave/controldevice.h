#ifndef CONTROLDEVICE_H
#define CONTROLDEVICE_H

#include <ros/ros.h>

#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/Joy.h"
#include <string>
#include <vector>

class ControlDevice {
public:
    ControlDevice(ros::NodeHandle& nh);
    ~ControlDevice();


private:
    void controlDeviceCallback(const sensor_msgs::Joy::ConstPtr& joy);
    ros::NodeHandle nh_;
    ros::Subscriber deviceSub;
    ros::Publisher  apiSub;
    double rotGain;
    double transGain;
    std::string curDeviceType;
    int curDeviceNum;



};


#endif // CONTROLDEVICE_H
