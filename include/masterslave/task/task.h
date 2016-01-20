#ifndef TASK_H
#define TASK_H

#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include "Eigen/Dense"
#include "masterslave/descriptionparameters.h"
#include "masterslave/kinematic/kinematics.h"
#include "sensor_msgs/JointState.h"

#include <dynamic_reconfigure/server.h>
#include <masterslave/masterslaveConfig.h>


class Task
{
    public:
        virtual Eigen::Affine3d moveEEFrame(Eigen::Affine3d)=0;
        void commandVelocities();
        void setGripperStatus(bool open, bool close){ gripper_open = open; gripper_close = close;}

    protected:
        double apertureLimit;
        double heightSafety;
        void calcQ6();

        virtual void loop()=0;

        ros::Subscriber Q4StateSub;
        ros::Subscriber Q5StateSub;
        ros::Subscriber Q6pStateSub;
        ros::Subscriber Q6nStateSub;

        ros::Publisher  Q4Pub;
        ros::Publisher  Q5Pub;
        ros::Publisher  Q6pPub;
        ros::Publisher  Q6nPub;

        toolMotorAngles motorAngles;
        toolAngles      toolAnglesAct;
        toolAngles      toolAnglesTar;

        Kinematics*     kinematic;

        bool gripper_stop;
        bool gripper_open;
        bool gripper_close;

        double cycleTime;
        double gripperVelocityValue;

        void configurationCallback(masterslave::masterslaveConfig &config, uint32_t level);
        Eigen::Quaternion<double> QuaternionFromEuler(const Eigen::Vector3d &eulerXYZ, bool ZYX);

};

#endif // TASK_H
