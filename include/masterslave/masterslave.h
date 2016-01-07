#ifndef MASTERSLAVE_H
#define MASTERSLAVE_H

#include <ros/ros.h>
#include <ros/console.h>
#include <sstream>
#include "masterslave/Button.h"
#include <faulhaber_driver/state.h>
#include "masterslave/laparoscope.h"
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <Eigen/Geometry>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>

#include <dynamic_reconfigure/server.h>
#include <masterslave/masterslaveConfig.h>

enum OPENIGTL_STATE
{
    IDLE,
    FREE,
    MASTERSLAVE
};

class MasterSlave
{
    public:
        MasterSlave(ros::NodeHandle&,ros::NodeHandle&);

    private:
        void flangeCallback(const geometry_msgs::PoseStampedConstPtr&);
        void velocityCallback(const geometry_msgs::TwistStampedConstPtr&);
        void buttonCallback(const masterslave::ButtonConstPtr&);
        void Q4StateCallback(const sensor_msgs::JointStateConstPtr &state);
        void Q5StateCallback(const sensor_msgs::JointStateConstPtr &state);
        void Q6nStateCallback(const sensor_msgs::JointStateConstPtr &state);
        void Q6pStateCallback(const sensor_msgs::JointStateConstPtr &state);
        void tcpCallback(const geometry_msgs::PoseStampedConstPtr &pose);
        void configurationCallback(masterslave::masterslaveConfig &config, uint32_t level);
        Eigen::Affine3d moveEEFrame(Eigen::Affine3d);
        void getTargetAngles(Laparoscope*);
        Eigen::Quaternion<double> QuaternionFromEuler(const Eigen::Vector3d &eulerXYZ, bool ZYX);
        tf::StampedTransform lookupROSTransform(const std::string from, const std::string to);

        void calcQ6();
        void commandVelocities();
        void buttonCheck(void);
        void doWorkRobot();

        ros::NodeHandle globalNH;
        ros::NodeHandle controlDeviceNH;

        ros::Subscriber startSub;
        ros::Subscriber stopSub;
        ros::Subscriber tcpSub;
        ros::Subscriber velocitySub;
        ros::Subscriber buttonSub;
        ros::Subscriber Q4StateSub;
        ros::Subscriber Q5StateSub;
        ros::Subscriber Q6pStateSub;
        ros::Subscriber Q6nStateSub;
        ros::Subscriber flangeSub;

        ros::Publisher  Q4Pub;
        ros::Publisher  Q5Pub;
        ros::Publisher  Q6pPub;
        ros::Publisher  Q6nPub;
        ros::Publisher  flangeTargetPub;
        ros::Publisher  rcmPub;
        ros::Publisher  statePub;


        Eigen::Affine3d lbrFlange;
        Eigen::Affine3d tcpAct;
        Eigen::Vector3d shaftTop;
        Eigen::Vector3d shaftBottom;
        Eigen::Vector3d rcm;

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
        double rosRate;
        double heightSafety;
        int apertureLimit;

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
        OPENIGTL_STATE state;


};

#endif // MASTERSLAVE_H
