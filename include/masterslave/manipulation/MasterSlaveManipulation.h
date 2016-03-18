#ifndef MASTERSLAVEMANIPULATION_H
#define MASTERSLAVEMANIPULATION_H

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/Float64.h"

#include "Eigen/Core"

#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>

#include "masterslave/Manipulation.h"

class MasterSlaveManipulation
{
    public:
        MasterSlaveManipulation(ros::NodeHandle& nh);
    private:
        void getControlDevice();
        bool masterSlaveCallback(masterslave::Manipulation::Request& req, masterslave::Manipulation::Response& resp);
        void velocityCallback(const geometry_msgs::TwistStampedConstPtr&);
        void cycleTimeCallback(const std_msgs::Float64ConstPtr&);
        Eigen::Quaternion<double> QuaternionFromEuler(const Eigen::Vector3d &eulerXYZ, bool ZYX);
        ros::NodeHandle nh_;
        ros::ServiceServer masterSlaveServer;
        ros::Subscriber velocitySub;
        ros::Subscriber cycleTimeSub;
        geometry_msgs::TwistStamped velocity;
        double cycleTime;

};

#endif // MASTERSLAVEMANIPULATION_H
