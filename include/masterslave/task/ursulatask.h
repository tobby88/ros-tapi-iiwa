#ifndef URSULATASK_H
#define URSULATASK_H

#include "task.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"

#include "masterslave/UrsulaRCM.h"
#include "masterslave/UrsulaDirectKinematics.h"
#include "masterslave/UrsulaInverseKinematics.h"


#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>

class UrsulaTask: public Task
{
    public:
        UrsulaTask(ros::NodeHandle& nh, double rosRate);
    private:
        ros::NodeHandle nh_;
        double rosRate_;

        void buttonCheck();
        void getControlDevice();
        void calcQ6();
        void commandVelocities();
        std::vector<std::string> buttons;

        ros::Publisher lbrJointAnglePub[7];
        ros::Publisher cycleTimePub;
        ros::Subscriber lbrJointAngleSub[7];

        void lbrJointAngleCallback(const sensor_msgs::JointStateConstPtr& state, int number);

        geometry_msgs::TwistStamped velocity_;

        void Q4StateCallback(const sensor_msgs::JointStateConstPtr &state);
        void Q5StateCallback(const sensor_msgs::JointStateConstPtr &state);
        void Q6nStateCallback(const sensor_msgs::JointStateConstPtr &state);
        void Q6pStateCallback(const sensor_msgs::JointStateConstPtr &state);
        void buttonCallback(const masterslave::ButtonConstPtr&);
        void flangeCallback(const geometry_msgs::PoseStampedConstPtr&);
        Eigen::Affine3d startPositionLBR;
        void loop();

        Eigen::VectorXd lbrJointAngles;
        Eigen::Affine3d RCM;
};

#endif // URSULATASK_H
