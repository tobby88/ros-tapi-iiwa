#include "masterslave/MasterSlave.h"
#include <ros/ros.h>


#define MM_TO_M 1/1000
#define DEG_TO_RAD M_PI/180

MasterSlave::MasterSlave(ros::NodeHandle& controlDeviceNH, ros::NodeHandle& taskNodeHandle): nh_(controlDeviceNH), taskNH_(taskNodeHandle)
{
    dynamic_reconfigure::Server<masterslave::MasterSlaveConfig> server(taskNH_);
    dynamic_reconfigure::Server<masterslave::MasterSlaveConfig>::CallbackType f;

    f = boost::bind(&MasterSlave::configurationCallback,this,_1,_2);

    server.setCallback(f);

    // Winkel werden in Rad/s übergeben (siehe server.cpp)
    stateService = nh_.serviceClient<masterslave::OpenIGTLStateService>("/openIGTLState");

    statemachineIsRunning = true;
    ros::Timer timer = nh_.createTimer(ros::Duration(0.02), &MasterSlave::statemachineThread, this);
    ros::spin();

}

MasterSlave::~MasterSlave()
{

}

void MasterSlave::statemachineThread(const ros::TimerEvent& event)
{
    masterslave::OpenIGTLStateService stateStringMsg;
    if(newState!=curState)
    {
        switch(newState)
        {
            case IDLE:
                if(stateService.exists()) stateStringMsg.request.state = "Idle;";
                curState = newState;
                break;
            case FREE:
                if(stateService.exists())  stateStringMsg.request.state = "Free;";
                curState = newState;
                break;
            case MOVE_TO_POSE:
                if(stateService.exists()) stateStringMsg.request.state = "MoveToPose;rob;";
                curState = newState;
                break;
        }
    }
    if(stateService.exists())
    {
        stateService.call(stateStringMsg);
        statemachineIsRunning = stateStringMsg.response.alive;
        ROS_DEBUG_STREAM("Service Call: alive: " << statemachineIsRunning);
    }


}

void MasterSlave::configurationCallback(masterslave::MasterSlaveConfig &config, uint32_t level)
{
    newState = static_cast<OPENIGTL_STATE>(config.cur_state);
    rosRate = config.rosRate;
    ROS_INFO_STREAM("current State: " << curState << " new State: " << newState);
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"MasterSlave");

    ros::NodeHandle nodeHandle("dynamicReconfigure");
    ros::NodeHandle ControlDeviceNH(argv[1]);

    MasterSlave MasterSlaveControl(ControlDeviceNH, nodeHandle);

    return 0;
}
