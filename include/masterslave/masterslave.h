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
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <Eigen/Geometry>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>

class MasterSlave
{
    public:
        MasterSlave(ros::NodeHandle&,ros::NodeHandle&);

    private:
        void flangeCallback(const geometry_msgs::TransformStampedConstPtr&);
        void velocityCallback(const geometry_msgs::TwistStampedConstPtr&);
        void startCallback(const std_msgs::BoolConstPtr&);
        void stopCallback(const std_msgs::BoolConstPtr&);
        void buttonCallback(const masterslave::ButtonConstPtr&);
        void Q4StateCallback(const sensor_msgs::JointStateConstPtr &state);
        void Q5StateCallback(const sensor_msgs::JointStateConstPtr &state);
        void Q6nStateCallback(const sensor_msgs::JointStateConstPtr &state);
        void Q6pStateCallback(const sensor_msgs::JointStateConstPtr &state);
        Eigen::Affine3d moveEEFrame(Eigen::Affine3d);
        void getTargetAngles(LaprascopicTool*);
        Eigen::Quaternion<double> QuaternionFromEuler(const Eigen::Vector3d &eulerXYZ, bool ZYX);
        tf::StampedTransform lookupROSTransform(const std::string from, const std::string to);

        void calcQ6();
        void commandVelocities();
        void buttonCheck(void);
        void doWorkTool(void);
        void doWorkRobot();

        ros::NodeHandle globalNH;
        ros::NodeHandle controlDeviceNH;

        ros::Subscriber startSub;
        ros::Subscriber stopSub;
        ros::Subscriber flangeSub;
        ros::Subscriber velocitySub;
        ros::Subscriber buttonSub;
        ros::Subscriber Q4StateSub;
        ros::Subscriber Q5StateSub;
        ros::Subscriber Q6pStateSub;
        ros::Subscriber Q6nStateSub;

        ros::Publisher  Q4Pub;
        ros::Publisher  Q5Pub;
        ros::Publisher  Q6pPub;
        ros::Publisher  Q6nPub;
        ros::Publisher  flangeTargetPub;


        Eigen::Vector3d RemoteCenterOfMotion;
        Eigen::Affine3d lbrFlange;
        double Q4_act;
        double Q5_act;
        double Q6n_act;
        double Q6p_act;
        double Q6_act;
        double Q4_target;
        double Q5_target;
        double Q6_target;
        double cycleTime;
        double lastTime;

        std_msgs::Float64 Q4Vel;
        std_msgs::Float64 Q5Vel;
        std_msgs::Float64 Q6nVel;
        std_msgs::Float64 Q6pVel;
        double gripperVelocityValue;
        geometry_msgs::TwistStamped velocity_;


        bool gripper_close;
        bool gripper_open;
        bool gripper_stop;
        bool start_;
        bool stop_;

        std::vector<std::string> buttons;

        std::string mode;



};

#endif // MASTERSLAVE_H
