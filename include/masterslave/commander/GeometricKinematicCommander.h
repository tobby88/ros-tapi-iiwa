#ifndef GEOMETRICKINEMATICCOMMANDER_H
#define GEOMETRICKINEMATICCOMMANDER_H

#include "ICommander.h"
#include "masterslave/kinematic/GeometricKinematic.h"
#include "ros/ros.h"



#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>

class GeometricKinematicCommander : public ICommander
{
    public:
        GeometricKinematicCommander(ros::NodeHandle& nh, ros::NodeHandle& drNH);


    private:
        void calcQ6();
        void commandVelocities();

        void buttonCheck();
        void getControlDevice();
        std::vector<std::string> buttons;


        ros::Subscriber velocitySub;
        ros::Subscriber buttonSub;

        ros::Publisher lbrTargetPositionPub;
        geometry_msgs::TwistStamped velocity_;

        void flangeCallback(const geometry_msgs::PoseStampedConstPtr&);
        void loop();
        ros::NodeHandle nh_;
        void Q4StateCallback(const sensor_msgs::JointStateConstPtr &state);
        void Q5StateCallback(const sensor_msgs::JointStateConstPtr &state);
        void Q6nStateCallback(const sensor_msgs::JointStateConstPtr &state);
        void Q6pStateCallback(const sensor_msgs::JointStateConstPtr &state);
        void velocityCallback(const geometry_msgs::TwistStampedConstPtr&);
        void buttonCallback(const masterslave::ButtonConstPtr&);


        Eigen::Affine3d T_0_FL;



};


#endif // LAPAROSCOPETASK_H
