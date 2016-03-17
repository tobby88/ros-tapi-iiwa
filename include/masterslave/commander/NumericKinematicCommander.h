#ifndef URSULATASK_H
#define URSULATASK_H

#include <array>

#include "ICommander.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"

#include "masterslave/NumericKinematicRCM.h"
#include "masterslave/NumericKinematicDirectKinematics.h"
#include "masterslave/NumericKinematicInverseKinematics.h"


#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>

class NumericKinematicCommander: public ICommander
{
    public:
        NumericKinematicCommander(ros::NodeHandle& nh, ros::NodeHandle& drNH);
    private:

        void configurationCallback(masterslave::MasterSlaveConfig &config, uint32_t level);
        ros::NodeHandle nh_;
        void buttonCheck();
        void getControlDevice();
        void calcQ6();
        void commandVelocities();
        std::vector<std::string> buttons;

        std::array<ros::Publisher,7> lbrJointAnglePub;
        ros::Publisher cycleTimePub;
        std::array<ros::Subscriber,7> lbrJointAngleSub;

        void lbrJointAngleCallback(const sensor_msgs::JointStateConstPtr& state, int number);

        geometry_msgs::TwistStamped velocity_;

        void Q4StateCallback(const sensor_msgs::JointStateConstPtr &state);
        void Q5StateCallback(const sensor_msgs::JointStateConstPtr &state);
        void Q6nStateCallback(const sensor_msgs::JointStateConstPtr &state);
        void Q6pStateCallback(const sensor_msgs::JointStateConstPtr &state);
        void buttonCallback(const masterslave::ButtonConstPtr&);
        Eigen::Affine3d startPositionLBR;
        void loop();

        Eigen::VectorXd lbrJointAngles;

        int callBacksCalled{0};

};

#endif // URSULATASK_H
