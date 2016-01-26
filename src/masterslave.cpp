#include "masterslave/masterslave.h"
#include <ros/ros.h>


#define MM_TO_M 1/1000
#define DEG_TO_RAD M_PI/180

MasterSlave::MasterSlave(ros::NodeHandle& controlDeviceNH, ros::NodeHandle& taskNodeHandle): nh_(controlDeviceNH), taskNH_(taskNodeHandle)
{
    start_ = false;
    rosRate = 50;

    dynamic_reconfigure::Server<masterslave::masterslaveConfig> server(taskNH_);
    dynamic_reconfigure::Server<masterslave::masterslaveConfig>::CallbackType f;

    f = boost::bind(&MasterSlave::configurationCallback,this,_1,_2);

    server.setCallback(f);

    // Winkel werden in Rad/s übergeben (siehe server.cpp)
    stateService = nh_.serviceClient<masterslave::state>("/openIGTLState");
    statemachineIsRunning = true;
    ros::Timer timer = nh_.createTimer(ros::Duration(0.02), &MasterSlave::statemachineThread, this);
    ros::spin();

}

void MasterSlave::statemachineThread(const ros::TimerEvent& event)
{
    masterslave::state stateStringMsg;
    switch(curState)
    {
        case IDLE:
            stateStringMsg.request.state = "Idle;";
            break;
        case FREE:
            stateStringMsg.request.state = "Free;";
            break;
        case MASTERSLAVE_LAPAROSCOPE:
            stateStringMsg.request.state = "MoveToPose;rob;";
            task = new LaparoscopeTask(nh_,rosRate);
            break;
        case MASTERSLAVE_URSULA:
            if(curState==MASTERSLAVE_LAPAROSCOPE) break;
            stateStringMsg.request.state = "MoveToJointAngles;";
            // URSULA-Task
            break;
    }
    if(stateService.exists())
    {
        stateService.call(stateStringMsg);
        statemachineIsRunning = stateStringMsg.response.alive;
        ROS_DEBUG_STREAM("Service Call: alive: " << statemachineIsRunning);
    }


}

void MasterSlave::configurationCallback(masterslave::masterslaveConfig &config, uint32_t level)
{
    curState = static_cast<OPENIGTL_STATE>(config.cur_state);
    rosRate = config.rosRate;
    start_ = config.start;
    ROS_INFO_STREAM("current State: " << curState);
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"MasterSlave");

    ros::NodeHandle ControlDeviceNH(argv[1]);
    ros::NodeHandle nodeHandle;
    MasterSlave MasterSlaveControl(ControlDeviceNH, nodeHandle);

    return 0;
}
