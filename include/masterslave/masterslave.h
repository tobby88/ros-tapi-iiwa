#ifndef MASTERSLAVE_H
#define MASTERSLAVE_H

#include <ros/ros.h>
#include <ros/console.h>
#include <sstream>
#include "masterslave/Button.h"
#include <faulhaber_driver/state.h>
#include "masterslave/laprascopictool.h"
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include <Eigen/Geometry>

class MasterSlave
{
    public:
        MasterSlave(ros::NodeHandle&,ros::NodeHandle&);

    private:
        void velocityCallback(const geometry_msgs::TwistStampedConstPtr&);
        void buttonCallback(const masterslave::ButtonConstPtr&);
        void Q5StateCallback(const sensor_msgs::JointStateConstPtr &state);
        void Q6nStateCallback(const sensor_msgs::JointStateConstPtr &state);
        void Q6pStateCallback(const sensor_msgs::JointStateConstPtr &state);

        void buttonCheck(void);
        void doWorkTool(void);

        ros::NodeHandle globalNH;
        ros::NodeHandle controlDeviceNH;
        ros::Subscriber velocitySub;
        ros::Subscriber buttonSub;
        ros::Subscriber Q5StateSub;
        ros::Subscriber Q6pStateSub;
        ros::Subscriber Q6nStateSub;
        ros::Publisher  Q5Pub;
        ros::Publisher  Q6pPub;
        ros::Publisher  Q6nPub;


        Eigen::Vector3d RemoteCenterOfMotion;
        double Q5_act;
        double Q6n_act;
        double Q6p_act;

        std_msgs::Float32 Q5Vel;
        std_msgs::Float32 Q6nVel;
        std_msgs::Float32 Q6pVel;
        double gripperVelocityValue;
        geometry_msgs::TwistStamped velocity_;


        bool gripper_close;
        bool gripper_open;

        std::vector<std::string> buttons;

        std::string mode;

        LaprascopicTool* tool;


};

#endif // MASTERSLAVE_H
