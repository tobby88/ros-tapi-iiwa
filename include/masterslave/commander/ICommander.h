#ifndef I_COMMANDER_H
#define I_COMMANDER_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include "Eigen/Dense"
#include "Eigen/Core"

#include "masterslave/DescriptionParameters.h"
#include "masterslave/kinematic/IKinematic.h"
#include "masterslave/commander/BoundingBox.h"

#include "sensor_msgs/JointState.h"
#include "masterslave/Button.h"

#include "masterslave/Manipulation.h"

#include <dynamic_reconfigure/server.h>

#include <masterslave/MasterSlaveConfig.h>
#include <masterslave/BoundingBoxConfig.h>

// State Service MEssage
#include "masterslave/OpenIGTLStateService.h"

#include "masterslave/OpenIGTLStateDescription.h"


class ICommander
{
    public:
        void setGripperStatus(bool open, bool close){ gripper_open = open; gripper_close = close;}
        void configurationCallback(masterslave::MasterSlaveConfig &config, uint32_t level);


    protected:
        virtual void statemachineThread(const ros::TimerEvent&)=0;
        Eigen::Affine3d startPositionLBR;
        Eigen::Affine3d TCP;
        Eigen::Affine3d RCM;
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
        ros::Subscriber lbrPositionSub;
        ros::Subscriber buttonSub;

        ros::ServiceClient tcpClient;
        ros::ServiceClient rcmClient;
        ros::ServiceClient directKinematicsClient;
        ros::ServiceClient inverseKinematicsClient;
        ros::ServiceClient  stateService;

        // Publisher for the tool joints
        ros::Publisher  Q4Pub;
        ros::Publisher  Q5Pub;
        ros::Publisher  Q6pPub;
        ros::Publisher  Q6nPub;

        Eigen::VectorXd motorAngles;
        Eigen::VectorXd jointAnglesTar;
        Eigen::VectorXd jointAnglesAct;


        bool gripper_stop;
        bool gripper_open;
        bool gripper_close;

        double cycleTime;
        double gripperVelocityValue;

        Eigen::Quaternion<double> QuaternionFromEuler(const Eigen::Vector3d &eulerXYZ, bool ZYX);

        const double DEG_TO_RAD{M_PI/180};

        std::unique_ptr<BoundingBox> boundingBox;

        Eigen::Vector3d boundingBoxSize{0.18,0.18,0.18};
        double rcmDistance{0.05};

        int Q6CallbacksCalled{0};

        int rosRate{1000};

        bool start_{false};

        // flag that shows if the OpenIGTL-Statemachine is running
        bool statemachineIsRunning;

        // current state of the LBR iiwa
        OPENIGTL_STATE newState{NO_STATE};
        OPENIGTL_STATE state{NO_STATE};

};

#endif // TASK_H
