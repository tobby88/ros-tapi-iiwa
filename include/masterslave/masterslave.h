#ifndef MASTERSLAVE_H
#define MASTERSLAVE_H

#include <ros/ros.h>
#include <ros/console.h>
#include <sstream>
#include "masterslave/Button.h"
#include <faulhaber_driver/state.h>
#include "masterslave/task/task.h"
#include "masterslave/task/laparoscopetask.h"
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
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
        MasterSlave(ros::NodeHandle&);

    private:
        void tcpCallback(const geometry_msgs::PoseStampedConstPtr &pose);
        void configurationCallback(masterslave::masterslaveConfig &config, uint32_t level);

        void getTargetAngles(Laparoscope*);
        tf::StampedTransform lookupROSTransform(const std::string from, const std::string to);

        void calcQ6();

        void buttonCheck(void);
        void doWorkRobot();

        ros::NodeHandle globalNH;
        ros::NodeHandle nh_;

        ros::Subscriber startSub;
        ros::Subscriber stopSub;
        ros::Subscriber tcpSub;

        ros::Publisher  rcmPub;
        ros::Publisher  statePub;


        Eigen::Affine3d tcpAct;

        double cycleTime;
        double lastTime;
        double rosRate;
        double heightSafety;
        int apertureLimit;
        bool start_;

        geometry_msgs::TwistStamped velocity_;

        OPENIGTL_STATE state;

        double gripperVelocityValue;
        Task* task;


};

#endif // MASTERSLAVE_H
