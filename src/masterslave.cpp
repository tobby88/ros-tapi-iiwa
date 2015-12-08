#include "masterslave/masterslave.h"
#include <ros/ros.h>


#define MM_TO_M 1/1000
#define DEG_TO_RAD M_PI/180
#define ORIENTATIONBOOST 1
#define RADTORPM (2*M_PI)/60
#define TRANSMISSION_G 169
#define TRANSMISSION_Q5 8.26/6.79
#define TRANSMISSION_Q6N 8.26/5.92
#define TRANSMISSION_Q6P 8.26/6.66

MasterSlave::MasterSlave(ros::NodeHandle& masterSlaveNH, ros::NodeHandle& controlDeviceNH): globalNH(masterSlaveNH), controlDeviceNH(controlDeviceNH)
{
    gripper_close = false;
    gripper_open = false;
    start_ = false;
    stop_ = false;

    Q6_act = 0.00000;
    Q5_act = 0.00000;
    Q4_act = 0.00000;

    std::stringstream device_sstream;
    XmlRpc::XmlRpcValue deviceList;
    ROS_INFO("Namespace: %s",globalNH.getNamespace().c_str());
    globalNH.param<std::string>("/MasterSlave/Mode", mode,"Laparoscope");
    globalNH.param("/MasterSlave/gripper_vel",gripperVelocityValue,0.1);
    globalNH.param("/MasterSlave/ros_rate",rosRate,2500.0);
    globalNH.param("/MasterSlave/height_safety",heightSafety,0.05);
    // in degree
    globalNH.param("/MasterSlave/aperture_limit",apertureLimit,60);

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
    startSub = globalNH.subscribe("startMasterSlave",1,&MasterSlave::startCallback,this);
    stopSub = globalNH.subscribe("stopMasterSlave",1,&MasterSlave::stopCallback,this);
    flangeSub = globalNH.subscribe("flangeLBR",1,&MasterSlave::flangeCallback,this);

    Q4Pub = globalNH.advertise<std_msgs::Float64>("Q4/setPointVelocity",1);
    Q5Pub = globalNH.advertise<std_msgs::Float64>("Q5/setPointVelocity",1);
    Q6nPub = globalNH.advertise<std_msgs::Float64>("Q6N/setPointVelocity",1);
    Q6pPub = globalNH.advertise<std_msgs::Float64>("Q6P/setPointVelocity",1);
    flangeTargetPub = globalNH.advertise<geometry_msgs::PoseStamped>("flangeTarget",1);
    rcmPub = globalNH.advertise<geometry_msgs::PoseStamped>("/RCM",1);

    ROS_INFO("Mode: %s",mode.c_str());
    buttonCheck();
    if(mode=="Robot")
    {
        doWorkRobot();
    }
    else if (mode=="Laparoscope")
    {
        doWorkTool();
    }
}

void MasterSlave::doWorkTool()
{
    double gripperVelocity;
    while(ros::ok())
    {
        Q5Vel.data =velocity_.twist.angular.y*cycleTime;
        Q4Vel.data =velocity_.twist.angular.z*cycleTime;
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

        Q6nVel.data = (velocity_.twist.angular.x + gripperVelocity)*cycleTime;
        Q6pVel.data = (velocity_.twist.angular.x - gripperVelocity)*cycleTime;
        Q5Pub.publish(Q5Vel);
        Q4Pub.publish(Q4Vel);
        Q6nPub.publish(Q6nVel);
        Q6pPub.publish(Q6pVel);
        ros::spinOnce();
    }
}

void MasterSlave::doWorkRobot()
{
    bool first = true;
    Laparoscope* tool;
    geometry_msgs::Pose poseFL;
    Eigen::Affine3d TCPist;
    ros::Rate rate(rosRate);
    while(ros::ok())
    {
        ros::spinOnce();
        if(start_ && !stop_)
        {
            if(first)
            {
                tool = new Laparoscope(lbrFlange);
                tool->setAngles(Q4_act,Q5_act,Q6_act);
                first = false;
                TCPist = lbrFlange*tool->getT_FL_EE();
                geometry_msgs::PoseStamped RCM;
                tf::poseEigenToMsg(tool->getRCM(),RCM.pose);
                rcmPub.publish(RCM);
                rcm  = tool->getRCM().translation();
            }
            else
            {

                calcQ6();
                shaftBottom = tool->getT_0_Q4().translation();
                TCPist = moveEEFrame(TCPist);

                tool->setT_0_EE(TCPist);
                //ROS_INFO("Q5_calc: %f",tool->getQ6());
                getTargetAngles(tool);
                commandVelocities();             
                tf::poseEigenToMsg(tool->getT_0_FL(),poseFL);
                geometry_msgs::PoseStamped T_0_FL_msg;
                T_0_FL_msg.pose = poseFL;
                flangeTargetPub.publish(T_0_FL_msg);
            }
            rate.sleep();
        }
        else
        {
            first = true;
        }
    }
}

void MasterSlave::getTargetAngles(Laparoscope* tool)
{
    Q4_target = tool->getQ4();
    Q5_target = tool->getQ5();
    Q6_target = tool->getQ6();
}

void MasterSlave::commandVelocities()
{
    double gripperVelocity;
    Q4Vel.data = (Q4_target - Q4_act);
    Q5Vel.data = (Q5_target - Q5_act);
    if(gripper_close && !gripper_open && !gripper_stop)
    {
        gripperVelocity = gripperVelocityValue;
    }
    else if(gripper_open && !gripper_close && !gripper_stop)
    {
        gripperVelocity = -gripperVelocityValue;
    }
    else
    {
        gripperVelocity = 0;
    }
    //Hier ist noch ein Fehler ;) Achsen?
    Q6nVel.data = ((Q6_target-Q6_act) + gripperVelocity);
    Q6pVel.data = ((Q6_target-Q6_act) - gripperVelocity);

    Q4Pub.publish(Q4Vel);
    Q5Pub.publish(Q5Vel);
    Q6nPub.publish(Q6nVel);
    Q6pPub.publish(Q6pVel);

}

void MasterSlave::tcpCallback(const geometry_msgs::PoseStampedConstPtr &pose)
{
    tf::poseMsgToEigen(pose->pose,tcpAct);
}

void MasterSlave::startCallback(const std_msgs::BoolConstPtr &val)
{
    start_ = val->data;
}

void MasterSlave::stopCallback(const std_msgs::BoolConstPtr &val)
{
    stop_ = val->data;
}

void MasterSlave::flangeCallback(const geometry_msgs::PoseStampedConstPtr& flangePose)
{
    tf::poseMsgToEigen(flangePose->pose,lbrFlange);
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
    cycleTime = ros::Time::now().toSec() - lastTime;
    velocity_ = *velocity;
    lastTime = ros::Time::now().toSec();
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

void MasterSlave::calcQ6()
{
    Q6_act = (Q6p_act + Q6n_act) / 2;

    if(Q6p_act < Q6n_act)
        gripper_stop = true;
    else
        gripper_stop = false;
}

Eigen::Affine3d MasterSlave::moveEEFrame(Eigen::Affine3d oldFrame)
{
    Eigen::Vector3d rcm_shaftBottom = shaftBottom-rcm;
    // calculation of the aperture of the frustum
    double aperture = asin(sqrt(pow(rcm_shaftBottom[0],2)+pow(rcm_shaftBottom[1],2))/-rcm_shaftBottom[2]);
    // polar angle in the frustum plane
    double polarAngle = atan2(rcm_shaftBottom[1],rcm_shaftBottom[0]);

    Eigen::Affine3d newFrame;
    newFrame.setIdentity();
    newFrame.translate(oldFrame.translation());

    if(aperture<=apertureLimit*DEG_TO_RAD || (velocity_.twist.linear.x/cos(polarAngle)<0 || velocity_.twist.linear.y/sin(polarAngle)<0))
    {
        newFrame.translate(Eigen::Vector3d(velocity_.twist.linear.x,velocity_.twist.linear.y,0));
    }

    if((shaftBottom[2]+heightSafety<rcm[2] || velocity_.twist.linear.z <0))
    {
        newFrame.translate(Eigen::Vector3d(0,0,velocity_.twist.linear.z));
    }


    newFrame.rotate(QuaternionFromEuler(Eigen::Vector3d(velocity_.twist.angular.x,velocity_.twist.angular.y,velocity_.twist.angular.z),true));
    newFrame.rotate(oldFrame.rotation());
    return newFrame;
}

Eigen::Quaternion<double> MasterSlave::QuaternionFromEuler(const Eigen::Vector3d &eulerXYZ, bool ZYX=true)

{
    Eigen::Quaternion<double> quat;
    quat.Identity();
    Eigen::AngleAxisd zAngle(eulerXYZ[2], Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yAngle(eulerXYZ[1], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd xAngle(eulerXYZ[0], Eigen::Vector3d::UnitX());
    if(ZYX)
        quat = zAngle * yAngle * xAngle;
    else
        quat = xAngle * yAngle * zAngle;

    return quat;
}

tf::StampedTransform MasterSlave::lookupROSTransform(const std::string from, const std::string to)
{
    tf::StampedTransform transform;
    tf::TransformListener listener_;
    try
    {
        ros::Time now = ros::Time(0);
        listener_.waitForTransform(from,
                                   to,
                                   now,
                                   ros::Duration(5.0));
        listener_.lookupTransform(from,
                                  to,
                                  now, transform);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

    return transform;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"MasterSlave");
    ros::NodeHandle MasterSlaveNH;
    ros::NodeHandle ControlDeviceNH(MasterSlaveNH, argv[1]);
    MasterSlave MasterSlaveControl(MasterSlaveNH,ControlDeviceNH);

    return 0;
}
