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
#include <masterslave/ControlDeviceConfig.h>


class ControlDevice {
public:
    ControlDevice(ros::NodeHandle& nh);
    void configurationCallback(masterslave::ControlDeviceConfig &config, uint32_t level);
    ~ControlDevice();


private:
    void controlDeviceCallback(const sensor_msgs::Joy::ConstPtr& joy);
    void registration();
    void buttonCheck();

    //flag if error is shown one time
    bool errorShown;


    ros::NodeHandle globalNH_;
    ros::NodeHandle nh_;

    //Subscriber for the device topic (gamepad, spacenav or marker device)
    ros::Subscriber deviceSub;

    //Publisher for the six axis values
    ros::Publisher  axisPub;

    //Publisher for the buttons, which are configured
    ros::Publisher  buttonsPub;

    //Map of configured and registred buttons
    std::map<int, std::string> buttons;

    //Gains for the Control Device
    double rotGain;
    double transGain;

    // Threshold for the input filtering
    double joyThresh;

    // used device type (gamepad, spacenav, or marker device)
    std::string curDeviceType;

    std::string apiDevice;

    //number of the device
    int curDeviceNum;

    //old joystick data to check if the joystick values have changed
    sensor_msgs::Joy joy_old;



};


#endif // CONTROLDEVICE_H
