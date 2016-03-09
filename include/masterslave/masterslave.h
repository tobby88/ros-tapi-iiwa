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
#include <masterslave/masterslaveConfig.h>

// Two Different Tasks and the Template for Inheritence
#include "masterslave/task/task.h"
#include "masterslave/task/laparoscopetask.h"
#include "masterslave/task/ursulatask.h"

// State Service MEssage
#include "masterslave/state.h"


// important and used states for the OpenIGTLink-Communication @see rosopenigtlbridge.h
enum OPENIGTL_STATE
{
    NOSTATE = -1,
    IDLE, //Idle state
    FREE, //Gravitation compensation
    MASTERSLAVE_LAPAROSCOPE, // first kinematic approach
    MASTERSLAVE_URSULA // second kinematic approach
};

class MasterSlave
{
    public:
        MasterSlave(ros::NodeHandle&,ros::NodeHandle&);
        ~MasterSlave();

    private:
        void configurationCallback(masterslave::masterslaveConfig &config, uint32_t level);

        void statemachineThread(const ros::TimerEvent&);

        ros::NodeHandle taskNH_;
        ros::NodeHandle nh_;

        ros::Publisher  rcmPub;
        ros::ServiceClient  stateService;

        double rosRate;

        bool start_;

        // flag that shows if the OpenIGTL-Statemachine is running
        bool statemachineIsRunning;

        // current state of the LBR iiwa
        OPENIGTL_STATE newState;
        OPENIGTL_STATE curState;

        //pointer on the current task-instance
        std::unique_ptr<Task> task;
        int taskCounter;


};

#endif // MASTERSLAVE_H
