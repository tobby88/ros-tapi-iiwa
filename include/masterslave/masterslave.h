#ifndef MASTERSLAVE_H
#define MASTERSLAVE_H

#include <ros/ros.h>
#include <ros/console.h>
#include <sstream>
#include "masterslave/Button.h"
#include "masterslave/state.h"
#include <faulhaber_driver/state.h>
#include "masterslave/task/task.h"
#include "masterslave/task/laparoscopetask.h"
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <Eigen/Geometry>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>

#include <dynamic_reconfigure/server.h>
#include <masterslave/masterslaveConfig.h>


// important and used states for the OpenIGTLink-Communication @see rosopenigtlbridge.h
enum OPENIGTL_STATE
{
    IDLE, //Idle state
    FREE, //Gravitation compensation
    MASTERSLAVE_LAPAROSCOPE, // first kinematic approach
    MASTERSLAVE_URSULA // second kinematic approach
};

class MasterSlave
{
    public:
        MasterSlave(ros::NodeHandle&,ros::NodeHandle&);

    private:
        void tcpCallback(const geometry_msgs::PoseStampedConstPtr &pose);
        void configurationCallback(masterslave::masterslaveConfig &config, uint32_t level);

        void getTargetAngles(Laparoscope*);
        tf::StampedTransform lookupROSTransform(const std::string from, const std::string to);

        void statemachineThread(const ros::TimerEvent&);

        ros::NodeHandle taskNH_;
        ros::NodeHandle nh_;

        ros::Publisher  rcmPub;
        ros::ServiceClient  stateService;


        double cycleTime;
        double lastTime;
        double rosRate;

        bool start_;
        // flag that shows if the OpenIGTL-Statemachine is running
        bool statemachineIsRunning;

        // current state of the LBR iiwa
        OPENIGTL_STATE curState;

        double gripperVelocityValue;

        //pointer on the current task-instance
        Task* task;


};

#endif // MASTERSLAVE_H
