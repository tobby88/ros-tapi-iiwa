#include "masterslave/task/laparoscopetask.h"

LaparoscopeTask::LaparoscopeTask(ros::NodeHandle &nh,double rosRate):rosRate_(rosRate), nh_(nh)
{
    //dynamic_reconfigure::Server<masterslave::kinematicConfig> server;
    //dynamic_reconfigure::Server<masterslave::kinematicConfig>::CallbackType f;

    //f = boost::bind(&LaparoscopeTask::configurationCallback,this,_1,_2);

    //server.setCallback(f);


    std::stringstream device_sstream;
    XmlRpc::XmlRpcValue deviceList;

    // TODO: Laparoskop-Kinematik einbinden
    if(strcmp(nh_.getNamespace().c_str(),"/Joy")==0)
    {
       nh_.getParam("/API/Joy",deviceList);
    }
    else if(strcmp(nh_.getNamespace().c_str(),"/Spacenav")==0)
    {
        nh_.getParam("/API/Spacenav",deviceList);
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
        velocitySub = nh_.subscribe(device_sstream.str().c_str(),10,&LaparoscopeTask::velocityCallback,this);
        device_sstream.str(std::string());
        device_sstream << it->first << "/Buttons";
        buttonSub = nh_.subscribe(device_sstream.str().c_str(),10,&LaparoscopeTask::buttonCallback, this);
        device_sstream.str(std::string());
    }

    Q4StateSub = nh_.subscribe("/Q4/joint_states",1,&LaparoscopeTask::Q4StateCallback, this);
    Q5StateSub = nh_.subscribe("/Q5/joint_states",1,&LaparoscopeTask::Q5StateCallback, this);
    Q6nStateSub = nh_.subscribe("/Q6N/joint_states",1,&LaparoscopeTask::Q6nStateCallback, this);
    Q6pStateSub = nh_.subscribe("/Q6P/joint_states",1,&LaparoscopeTask::Q6pStateCallback, this);
    lbrPositionSub = nh_.subscribe("/flangeLBR",1,&LaparoscopeTask::flangeCallback, this);

    Q4Pub = nh_.advertise<std_msgs::Float64>("/Q4/setPointVelocity",1);
    Q5Pub = nh_.advertise<std_msgs::Float64>("/Q5/setPointVelocity",1);
    Q6nPub = nh_.advertise<std_msgs::Float64>("/Q6N/setPointVelocity",1);
    Q6pPub = nh_.advertise<std_msgs::Float64>("/Q6P/setPointVelocity",1);
    lbrTargetPositionPub = nh_.advertise<geometry_msgs::PoseStamped>("/flangeTarget",1);


    ros::Rate waiteRate(0.5);
    while(ros::ok() && !kinematic)
    {
        ROS_INFO("LaparoscopeTask is waiting for a start position of the robot");
        ros::spinOnce();
        waiteRate.sleep();
    }
    ROS_INFO_COND(kinematic,"LaroscopeTask has found the start position of the robot!");


}

void LaparoscopeTask::flangeCallback(const geometry_msgs::PoseStampedConstPtr& flangePose)
{

    tf::poseMsgToEigen(flangePose->pose,startPositionLBR);
    kinematic = new Laparoscope(startPositionLBR);

    lbrPositionSub.shutdown();
    loop();
}

Eigen::Affine3d LaparoscopeTask::moveEEFrame(Eigen::Affine3d oldFrame)
{
    Eigen::Vector3d shaftBottom = kinematic->getT_0_Q4().translation();
    Eigen::Vector3d rcm = kinematic->getRCM().translation();
    Eigen::Vector3d rcm_shaftBottom = shaftBottom -rcm;
    // calculation of the aperture of the frustum
    double aperture = asin(sqrt(pow(rcm_shaftBottom[0],2)+pow(rcm_shaftBottom[1],2))/rcm_shaftBottom[2]);

    // polar angle in the frustum plane
    double polarAngle = atan2(rcm_shaftBottom[1],rcm_shaftBottom[0]);

    Eigen::Affine3d newFrame;
    newFrame.setIdentity();
    newFrame.translate(oldFrame.translation());

    if(aperture<=apertureLimit*DEG_TO_RAD || (velocity_.twist.linear.x/cos(polarAngle)<0 || velocity_.twist.linear.y/sin(polarAngle)<0))
    {
        newFrame.translate(Eigen::Vector3d(velocity_.twist.linear.x*cycleTime,velocity_.twist.linear.y*cycleTime,0));
    }

    if((shaftBottom[2]+heightSafety<rcm[2] || velocity_.twist.linear.z <0) && (oldFrame.translation().z() > heightSafety || velocity_.twist.linear.z > 0))
    {
        newFrame.translate(Eigen::Vector3d(0,0,velocity_.twist.linear.z*cycleTime));
    }

    newFrame.rotate(QuaternionFromEuler(Eigen::Vector3d(velocity_.twist.angular.x*cycleTime,velocity_.twist.angular.y*cycleTime,velocity_.twist.angular.z*cycleTime),true));
    newFrame.rotate(oldFrame.rotation());
    //Plausibilitätskontrolle
    return newFrame;
}


void LaparoscopeTask::loop()
{
    double lastTime = ros::Time::now().toSec();
    kinematic->setAngles(toolAnglesAct);
    TCP = startPositionLBR*kinematic->getT_FL_EE();
    geometry_msgs::PoseStamped poseFLmsg;
    while(ros::ok())
    {
        ros::spinOnce();
        cycleTime = ros::Time::now().toSec() - lastTime;
        lastTime = ros::Time::now().toSec();
        kinematic->setT_0_EE(TCP);
        TCP = moveEEFrame(TCP);
        kinematic->getAngles();
        commandVelocities();
        tf::poseEigenToMsg(kinematic->getT_0_FL(),poseFLmsg.pose);
        lbrTargetPositionPub.publish(poseFLmsg);

    }
    ros::shutdown();

}


void LaparoscopeTask::Q4StateCallback(const sensor_msgs::JointStateConstPtr &state)
{
    motorAngles.Q4 = state->position.at(0);
}


void LaparoscopeTask::Q5StateCallback(const sensor_msgs::JointStateConstPtr &state)
{
    motorAngles.Q5 = state->position.at(0);
}

void LaparoscopeTask::Q6nStateCallback(const sensor_msgs::JointStateConstPtr &state)
{
    motorAngles.Q6N= state->position.at(0);
}

void LaparoscopeTask::Q6pStateCallback(const sensor_msgs::JointStateConstPtr &state)
{
    motorAngles.Q6P= state->position.at(0);
}

void LaparoscopeTask::buttonCallback(const masterslave::ButtonConstPtr &button)
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

void LaparoscopeTask::velocityCallback(const geometry_msgs::TwistStampedConstPtr &velocity)
{

    velocity_ = *velocity;

}



void LaparoscopeTask::buttonCheck()
{
    std::stringstream button_sstream;
    button_sstream << "button";
    if(nh_.hasParam(button_sstream.str().c_str()))
    {
        XmlRpc::XmlRpcValue buttonList;
        nh_.getParam(button_sstream.str().c_str(),buttonList);
        for(XmlRpc::XmlRpcValue::iterator it=buttonList.begin();it!=buttonList.end();it++)
        {
            buttons.push_back((std::string&)(it->second));
        }

    }
}
