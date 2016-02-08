#ifndef TASK_H
#define TASK_H

#include "ros/ros.h"
#include <std_msgs/Float64.h>

#include "Eigen/Dense"
#include "Eigen/Core"

#include "masterslave/descriptionparameters.h"
#include "masterslave/kinematic/kinematics.h"

#include "sensor_msgs/JointState.h"
#include <geometry_msgs/TwistStamped.h>
#include "masterslave/Button.h"


#include <dynamic_reconfigure/server.h>

#include <masterslave/kinematicConfig.h>


class Task
{
    public:
        virtual Eigen::Affine3d moveEEFrame(Eigen::Affine3d)=0;

        void setGripperStatus(bool open, bool close){ gripper_open = open; gripper_close = close;}

    protected:
        Eigen::Affine3d startPositionLBR;
        Eigen::Affine3d TCP;
        double apertureLimit;
        double heightSafety;
        virtual void calcQ6()=0;
        virtual void commandVelocities()=0;
        virtual void loop()=0;

        // Subscriber for the four joint states
        ros::Subscriber Q4StateSub;
        ros::Subscriber Q5StateSub;
        ros::Subscriber Q6pStateSub;
        ros::Subscriber Q6nStateSub;

        //spacenav subscribers for the communication with "ControlDevice-Node"
        ros::Subscriber velocitySub;
        ros::Subscriber buttonSub;
        ros::Subscriber lbrPositionSub;

        // Publisher for the tool joints
        ros::Publisher  Q4Pub;
        ros::Publisher  Q5Pub;
        ros::Publisher  Q6pPub;
        ros::Publisher  Q6nPub;

        Eigen::VectorXd motorAngles;
        Eigen::VectorXd jointAnglesTar;
        Eigen::VectorXd jointAnglesAct;

        Kinematics*     kinematic;

        bool gripper_stop;
        bool gripper_open;
        bool gripper_close;

        double cycleTime;
        double gripperVelocityValue;

        void configurationCallback(masterslave::kinematicConfig &config, uint32_t level);
        Eigen::Quaternion<double> QuaternionFromEuler(const Eigen::Vector3d &eulerXYZ, bool ZYX);

        static const double DEG_TO_RAD;

};

#endif // TASK_H
