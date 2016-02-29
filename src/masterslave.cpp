#include "masterslave/masterslave.h"
#include <ros/ros.h>


#define MM_TO_M 1/1000
#define DEG_TO_RAD M_PI/180

MasterSlave::MasterSlave(ros::NodeHandle& controlDeviceNH, ros::NodeHandle& taskNodeHandle): nh_(controlDeviceNH), taskNH_(taskNodeHandle)
{
    start_ = false;
    rosRate = 50;
    taskCounter = 0;
    curState = NOSTATE;

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
    if(newState!=curState)
    {
        if(taskCounter>0)
        {
            delete task;
            taskCounter = 0;
        }
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
            case MASTERSLAVE_LAPAROSCOPE:
                if(stateService.exists()) stateStringMsg.request.state = "MoveToPose;rob;";
                task = new LaparoscopeTask(nh_,rosRate);
                curState = newState;
                taskCounter++;
                break;
            case MASTERSLAVE_URSULA:
                if(stateService.exists()) if(curState==MASTERSLAVE_LAPAROSCOPE) break;
                stateStringMsg.request.state = "MoveToJointAngles;";
                task = new UrsulaTask(nh_,rosRate);
                curState = newState;
                taskCounter++;
                // URSULA-Task
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

void MasterSlave::configurationCallback(masterslave::masterslaveConfig &config, uint32_t level)
{
    newState = static_cast<OPENIGTL_STATE>(config.cur_state);
    rosRate = config.rosRate;
    ROS_INFO_STREAM("current State: " << curState << " new State: " << newState);
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"MasterSlave");

    ros::NodeHandle nodeHandle("~");
    ros::NodeHandle ControlDeviceNH(argv[1]);

    MasterSlave MasterSlaveControl(ControlDeviceNH, nodeHandle);

    return 0;
}
