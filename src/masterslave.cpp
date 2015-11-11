#include "masterslave/masterslave.h"
#include <ros/ros.h>


#define MM_TO_M 1/1000
#define ORIENTATIONBOOST 1
#define RADTORPM (2*M_PI)/60
#define TRANSMISSION_G 169
#define TRANSMISSION_Q5 8.26/6.79
#define TRANSMISSION_Q6N 8.26/5.92
#define TRANSMISSION_Q6P 8.26/6.66

MasterSlave::MasterSlave(ros::NodeHandle& masterSlaveNH, ros::NodeHandle& controlDeviceNH): globalNH(masterSlaveNH), controlDeviceNH(controlDeviceNH)
{
    std::stringstream device_sstream;
    XmlRpc::XmlRpcValue deviceList;

    globalNH.param<std::string>("Mode", mode,"Laparoscope");
    globalNH.param("gripper_vel",gripperVelocityValue,20.0);

    // TODO: Laparoskop-Kinematik einbinden
    if(strcmp(controlDeviceNH.getNamespace().c_str(),"/Joy")==0)
    {
       controlDeviceNH.getParam("/API/Joy",deviceList);
    }
    else if(strcmp(controlDeviceNH.getNamespace().c_str(),"/Spacenav")==0)
    {
        controlDeviceNH.getParam("/API/Spacenav",deviceList);
    }
    else
    {
        ROS_ERROR("No ControlDevice found!");
        return;
    }
    //TODO: Auswahl des gewünschten Steuerungsgeräts
    for(XmlRpc::XmlRpcValue::iterator it = deviceList.begin(); it!=deviceList.end();it++)
    {
        device_sstream << it->first << "/Velocity";
        velocitySub = controlDeviceNH.subscribe(device_sstream.str().c_str(),10,&MasterSlave::velocityCallback,this);
        device_sstream.str(std::string());
        device_sstream << it->first << "/Buttons";
        buttonSub = controlDeviceNH.subscribe(device_sstream.str().c_str(),10,&MasterSlave::buttonCallback, this);
        device_sstream.str(std::string());
    }
    // Winkel werden in Rad übergeben (siehe server.cpp)
    Q4StateSub = globalNH.subscribe("Q4/joint_states",1,&MasterSlave::Q4StateCallback, this);
    Q5StateSub = globalNH.subscribe("Q5/joint_states",1,&MasterSlave::Q5StateCallback, this);
    Q6nStateSub = globalNH.subscribe("Q6N/joint_states",1,&MasterSlave::Q6nStateCallback, this);
    Q6pStateSub = globalNH.subscribe("Q6P/joint_states",1,&MasterSlave::Q6pStateCallback, this);
    Q4Pub = globalNH.advertise<std_msgs::Float64>("Q4/setPointVelocity",1);
    Q5Pub = globalNH.advertise<std_msgs::Float64>("Q5/setPointVelocity",1);
    Q6nPub = globalNH.advertise<std_msgs::Float64>("Q6N/setPointVelocity",1);
    Q6pPub = globalNH.advertise<std_msgs::Float64>("Q6P/setPointVelocity",1);
    gripper_close = false;
    gripper_open = false;
    ROS_INFO("Mode: %s",mode.c_str());
    buttonCheck();
    if(mode=="Robot")
    {
        return;
    }
    else if (mode=="Laparoscope")
    {
        doWorkTool();
    }
}

void MasterSlave::doWorkTool()
{
        double gripperVelocity;
        Q5Vel.data =velocity_.twist.angular.y;
        Q4Vel.data =velocity_.twist.angular.z;
        if(gripper_close && !gripper_open)
        {
            gripperVelocity = gripperVelocityValue;
        }
        else if(gripper_open && !gripper_close)
        {
            gripperVelocity = -gripperVelocityValue;
        }
        else
        {
            gripperVelocity = 0;
        }

        Q6nVel.data = velocity_.twist.angular.x + gripperVelocity;
        Q6pVel.data = velocity_.twist.angular.x - gripperVelocity;
        Q5Pub.publish(Q5Vel);
        Q4Pub.publish(Q4Vel);
        Q6nPub.publish(Q6nVel);
        Q6pPub.publish(Q6pVel);
}

void MasterSlave::buttonCallback(const masterslave::ButtonConstPtr &button)
{
    //TODO: Buttonnamen vom Parameterserver holen
    if(button->name == "close_gripper" && button->state == 1)
    {
        // Greifer schließen
        gripper_close = true;
        gripper_open = false;
    }
    else if(button->name == "open_gripper" && button->state == 1)
    {
        // Greifer öffnen
        gripper_open = true;
        gripper_close = false;
    }
    else
    {
        gripper_open = false;
        gripper_close = false;
    }
}

void MasterSlave::velocityCallback(const geometry_msgs::TwistStampedConstPtr &velocity)
{
    velocity_ = *velocity;
    doWorkTool();
}

void MasterSlave::Q4StateCallback(const sensor_msgs::JointStateConstPtr &state)
{
    Q4_act = state->position.at(0);
}


void MasterSlave::Q5StateCallback(const sensor_msgs::JointStateConstPtr &state)
{
    Q5_act = state->position.at(0);
}

void MasterSlave::Q6nStateCallback(const sensor_msgs::JointStateConstPtr &state)
{
    Q6n_act = state->position.at(0);
}

void MasterSlave::Q6pStateCallback(const sensor_msgs::JointStateConstPtr &state)
{
    Q6p_act = state->position.at(0);
}

void MasterSlave::buttonCheck()
{
    std::stringstream button_sstream;
    button_sstream << "button";
    if(controlDeviceNH.hasParam(button_sstream.str().c_str()))
    {
        XmlRpc::XmlRpcValue buttonList;
        controlDeviceNH.getParam(button_sstream.str().c_str(),buttonList);
        for(XmlRpc::XmlRpcValue::iterator it=buttonList.begin();it!=buttonList.end();it++)
        {
            buttons.push_back((std::string&)(it->second));
        }

    }

}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"MasterSlave");
    ros::NodeHandle MasterSlaveNH;
    ros::NodeHandle ControlDeviceNH(MasterSlaveNH, argv[1]);
    MasterSlave MasterSlaveControl(MasterSlaveNH,ControlDeviceNH);
    ros::spin();
    return 0;
}
