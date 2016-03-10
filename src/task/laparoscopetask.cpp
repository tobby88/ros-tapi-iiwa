#include "masterslave/task/laparoscopetask.h"

LaparoscopeTask::LaparoscopeTask(ros::NodeHandle &nh,double rosRate):rosRate_(rosRate), nh_(nh)
{

    if(instances>0) return;
    dynamic_reconfigure::Server<masterslave::kinematicConfig> server(nh_);
    dynamic_reconfigure::Server<masterslave::kinematicConfig>::CallbackType f;

    f = boost::bind(&LaparoscopeTask::configurationCallback,this,_1,_2);

    server.setCallback(f);
    getControlDevice();
    Q4StateSub = nh_.subscribe("/Q4/joint_states",1,&LaparoscopeTask::Q4StateCallback, this);
    Q5StateSub = nh_.subscribe("/Q5/joint_states",1,&LaparoscopeTask::Q5StateCallback, this);
    Q6nStateSub = nh_.subscribe("/Q6N/joint_states",1,&LaparoscopeTask::Q6nStateCallback, this);
    Q6pStateSub = nh_.subscribe("/Q6P/joint_states",1,&LaparoscopeTask::Q6pStateCallback, this);
    lbrPositionSub = nh_.subscribe("/flangeLBR",1,&LaparoscopeTask::flangeCallback, this);

    rcmClient = nh_.serviceClient<masterslave::LaparoscopeRCM>("/RCM");
    directKinematicsClient = nh_.serviceClient<masterslave::LaparoscopeDirectKinematics>("/directKinematics");
    inverseKinematicsClient = nh_.serviceClient<masterslave::LaparoscopeInverseKinematics>("/inverseKinematics");
    tcpClient = nh_.serviceClient<masterslave::Manipulation>("/Manipulation");

    Q4Pub = nh_.advertise<std_msgs::Float64>("/Q4/setPointVelocity",1);
    Q5Pub = nh_.advertise<std_msgs::Float64>("/Q5/setPointVelocity",1);
    Q6nPub = nh_.advertise<std_msgs::Float64>("/Q6N/setPointVelocity",1);
    Q6pPub = nh_.advertise<std_msgs::Float64>("/Q6P/setPointVelocity",1);
    lbrTargetPositionPub = nh_.advertise<geometry_msgs::PoseStamped>("/flangeTarget",1);
    instances++;

    ros::Rate waiteRate(0.5);
    while(ros::ok() && lbrPositionSub.getNumPublishers()==1)
    {
        ROS_INFO("LaparoscopeTask is waiting for a start position of the robot");
        ros::spinOnce();
        waiteRate.sleep();
    }
    ROS_INFO("LaroscopeTask has found the start position of the robot!");
    if(lbrPositionSub.getNumPublishers()!=1)
    {
        loop();
    }


}

void LaparoscopeTask::getControlDevice()
{
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
        device_sstream << it->first << "/Buttons";
        buttonSub = nh_.subscribe(device_sstream.str().c_str(),10,&LaparoscopeTask::buttonCallback, this);
        device_sstream.str(std::string());
    }
}

void LaparoscopeTask::flangeCallback(const geometry_msgs::PoseStampedConstPtr& flangePose)
{
    tf::poseMsgToEigen(flangePose->pose,startPositionLBR);
    lbrPositionSub.shutdown();
}

void LaparoscopeTask::loop()
{
    double lastTime = ros::Time::now().toSec();
    masterslave::LaparoscopeRCM rcmService;
    tf::poseEigenToMsg(startPositionLBR,rcmService.request.T_0_FL);
    rcmClient.call(rcmService);

    masterslave::LaparoscopeDirectKinematics directKinematicsService;
    std::vector<double> jointAngles(jointAnglesAct.data(),jointAnglesAct.data()+jointAnglesAct.rows());
    directKinematicsService.request.jointAngles = jointAngles;
    tf::poseEigenToMsg(startPositionLBR,rcmService.request.T_0_FL);
    directKinematicsClient.call(directKinematicsService);

    ros::Rate rate(rosRate_);
    while(ros::ok())
    {
        ros::spinOnce();
        cycleTime = ros::Time::now().toSec() - lastTime;
        lastTime = ros::Time::now().toSec();

        // Hier muss der TCP-Service gecallt werden
        masterslave::Manipulation manipulationService;
        tf::poseEigenToMsg(TCP,manipulationService.request.T_0_EE_old);
        tcpClient.call(manipulationService);
        tf::poseMsgToEigen(manipulationService.response.T_0_EE_new,TCP);

        masterslave::LaparoscopeInverseKinematics inverseKinematicsService;
        tf::poseEigenToMsg(TCP, inverseKinematicsService.request.T_0_EE);
        inverseKinematicsClient.call(inverseKinematicsService);
        tf::poseMsgToEigen(inverseKinematicsService.response.T_0_FL,T_0_FL);
        lbrTargetPositionPub.publish(inverseKinematicsService.response.T_0_FL);
        jointAnglesTar = Eigen::VectorXd::Map(inverseKinematicsService.response.jointAnglesTarget.data(),inverseKinematicsService.response.jointAnglesTarget.size());
        rate.sleep();
    }
    ros::shutdown();
}


void LaparoscopeTask::Q4StateCallback(const sensor_msgs::JointStateConstPtr &state)
{
    motorAngles(0) = state->position.at(0);
}


void LaparoscopeTask::Q5StateCallback(const sensor_msgs::JointStateConstPtr &state)
{
    motorAngles(1) = state->position.at(0);
}

void LaparoscopeTask::Q6nStateCallback(const sensor_msgs::JointStateConstPtr &state)
{
    motorAngles(2) = state->position.at(0);
}

void LaparoscopeTask::Q6pStateCallback(const sensor_msgs::JointStateConstPtr &state)
{
    motorAngles(3)= state->position.at(0);
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

void LaparoscopeTask::calcQ6()
{
    jointAnglesAct(2) = (motorAngles(4) + motorAngles(3)) / 2;

    if(fabs(motorAngles(4) + motorAngles(3))<0)
        gripper_stop = true;
    else
        gripper_stop = false;
}

void LaparoscopeTask::commandVelocities()
{
    double gripperVelocity;
    std_msgs::Float64 Q4Vel, Q5Vel, Q6nVel, Q6pVel;
    Q4Vel.data = (jointAnglesTar(0) - jointAnglesAct(0))/cycleTime;
    Q5Vel.data = (jointAnglesTar(1) - jointAnglesAct(1))/cycleTime;
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

    Q6nVel.data = (jointAnglesTar(2)-jointAnglesAct(2))/cycleTime;
    Q6pVel.data = (jointAnglesTar(2)-jointAnglesAct(2))/cycleTime;
    // Stoppen der Greiferbacken, wenn eine der beiden am Anschlag ist, um Greiferöffnungswinkel nicht zu ändern
    if(motorAngles(4)>=0.95*M_PI && (jointAnglesTar(2)-jointAnglesAct(2))>0)
    {
        Q6nVel.data = 0;
    }
    if(motorAngles(3)<=-0.95*M_PI && (jointAnglesTar(2)-jointAnglesAct(2))<0)
    {
        Q6pVel.data = 0;
    }

    Q6nVel.data += gripperVelocity/cycleTime;
    Q6pVel.data -= gripperVelocity/cycleTime;

    Q4Pub.publish(Q4Vel);
    Q5Pub.publish(Q5Vel);
    Q6nPub.publish(Q6nVel);
    Q6pPub.publish(Q6pVel);
}




