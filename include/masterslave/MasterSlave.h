#ifndef MASTERSLAVE_H
#define MASTERSLAVE_H

#include <ros/ros.h>
#include <ros/console.h>
#include <sstream>
// My own Button Message for the Button Recognition
#include "masterslave/Button.h"

// Message of the Faulhaber Driver
#include <faulhaber_driver/state.h>


#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

// TF Stuff
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>

//Dynamic Reconfigure Stuff
#include <dynamic_reconfigure/server.h>
#include <masterslave/MasterSlaveConfig.h>

// State Service MEssage
#include "masterslave/OpenIGTLStateService.h"

#include "masterslave/OpenIGTLStateDescription.h"

class MasterSlave
{
    public:
        MasterSlave(ros::NodeHandle&,ros::NodeHandle&);
        ~MasterSlave();

    private:
        void configurationCallback(masterslave::MasterSlaveConfig &config, uint32_t level);

        void statemachineThread(const ros::TimerEvent&);

        ros::NodeHandle taskNH_;
        ros::NodeHandle nh_;

        ros::Publisher  rcmPub;
        ros::ServiceClient  stateService;

        double rosRate{1000};

        bool start_{false};

        // flag that shows if the OpenIGTL-Statemachine is running
        bool statemachineIsRunning;

        // current state of the LBR iiwa
        OPENIGTL_STATE newState;
        OPENIGTL_STATE curState{NO_STATE};

};

#endif // MASTERSLAVE_H
