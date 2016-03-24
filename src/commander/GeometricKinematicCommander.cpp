#include "masterslave/commander/GeometricKinematicCommander.h"

GeometricKinematicCommander::GeometricKinematicCommander(ros::NodeHandle &nh, ros::NodeHandle& drNH): nh_(nh)
{    
    dynamic_reconfigure::Server<masterslave::MasterSlaveConfig> server(drNH);
    dynamic_reconfigure::Server<masterslave::MasterSlaveConfig>::CallbackType f;

    jointAnglesAct = Eigen::Vector3d::Zero(3);
    jointAnglesTar = Eigen::Vector3d::Zero(3);
    motorAngles = Eigen::Vector2d::Zero();

    f = boost::bind(&ICommander::configurationCallback,this,_1,_2);

    server.setCallback(f);

    getControlDevice();
    Q4StateSub = nh_.subscribe("/Q4/joint_states",1,&GeometricKinematicCommander::Q4StateCallback, this);
    Q5StateSub = nh_.subscribe("/Q5/joint_states",1,&GeometricKinematicCommander::Q5StateCallback, this);
    Q6nStateSub = nh_.subscribe("/Q6N/joint_states",1,&GeometricKinematicCommander::Q6nStateCallback, this);
    Q6pStateSub = nh_.subscribe("/Q6P/joint_states",1,&GeometricKinematicCommander::Q6pStateCallback, this);
    lbrPositionSub = nh_.subscribe("/flangeLBR",1,&GeometricKinematicCommander::flangeCallback, this);

    rcmClient = nh_.serviceClient<masterslave::GeometricKinematicRCM>("/RCM");
    directKinematicsClient = nh_.serviceClient<masterslave::GeometricKinematicDirectKinematics>("/directKinematics");
    inverseKinematicsClient = nh_.serviceClient<masterslave::GeometricKinematicInverseKinematics>("/inverseKinematics");
    tcpClient = nh_.serviceClient<masterslave::Manipulation>("/Manipulation");
    stateService = nh_.serviceClient<masterslave::OpenIGTLStateService>("/openIGTLState");

    Q4Pub = nh_.advertise<std_msgs::Float64>("/Q4/setPointVelocity",1);
    Q5Pub = nh_.advertise<std_msgs::Float64>("/Q5/setPointVelocity",1);
    Q6nPub = nh_.advertise<std_msgs::Float64>("/Q6N/setPointVelocity",1);
    Q6pPub = nh_.advertise<std_msgs::Float64>("/Q6P/setPointVelocity",1);
    lbrTargetPositionPub = nh_.advertise<geometry_msgs::PoseStamped>("/flangeTarget",1);
    cycleTimePub = nh_.advertise<std_msgs::Float64>("/cycleTime",1);

    statemachineIsRunning = true;
    ros::Timer timer = nh_.createTimer(ros::Duration(0.02), &GeometricKinematicCommander::statemachineThread, this);

    ros::Rate waiteRate(0.5);
    while(ros::ok() && !(callBacksCalled+1 >> 4 == 1))
    {
        ROS_INFO_STREAM(nh_.getNamespace() <<" is waiting for a start position of the robot");
        ros::spinOnce();
        waiteRate.sleep();
    }
    ROS_INFO_STREAM(nh_.getNamespace() << " has found the start position of the robot!");
    while(ros::ok())
    {
        ros::spinOnce();
        ROS_INFO_STREAM(state);
        if(state == MOVE_TO_POSE)
        {
           loop();
        }

        waiteRate.sleep();
    }

}

void GeometricKinematicCommander::statemachineThread(const ros::TimerEvent& event)
{
    masterslave::OpenIGTLStateService stateStringMsg;
    if(newState!=state)
    {
        switch(newState)
        {
            case IDLE:
                if(stateService.exists()) stateStringMsg.request.state = "Idle;";
                state = newState;
                break;
            case FREE:
                if(stateService.exists())  stateStringMsg.request.state = "Free;";
                state = newState;
                break;
            case MOVE_TO_POSE:
                if(stateService.exists()) stateStringMsg.request.state = "MoveToPose;rob;";
                state = newState;
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

void GeometricKinematicCommander::getControlDevice()
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
        buttonSub = nh_.subscribe(device_sstream.str().c_str(),10,&GeometricKinematicCommander::buttonCallback, this);
        device_sstream.str(std::string());
    }
}

void GeometricKinematicCommander::flangeCallback(const geometry_msgs::PoseStampedConstPtr& flangePose)
{
    tf::poseMsgToEigen(flangePose->pose,startPositionLBR);
    callBacksCalled += 1 << 3;
    lbrPositionSub.shutdown();
}

void GeometricKinematicCommander::loop()
{
    double lastTime = ros::Time::now().toSec();
    masterslave::GeometricKinematicRCM rcmService;
    geometry_msgs::PoseStamped stampedPose;
    ROS_INFO_STREAM(startPositionLBR.matrix());
    tf::poseEigenToMsg(startPositionLBR,rcmService.request.T_0_FL);
    rcmClient.call(rcmService);

    masterslave::GeometricKinematicDirectKinematics directKinematicsService;
    std::vector<double> jointAngles(jointAnglesAct.data(),jointAnglesAct.data()+jointAnglesAct.rows());
    directKinematicsService.request.jointAngles = jointAngles;
    tf::poseEigenToMsg(startPositionLBR,directKinematicsService.request.T_0_FL);
    directKinematicsClient.call(directKinematicsService);
    tf::poseMsgToEigen(directKinematicsService.response.T_0_EE,TCP);
    ROS_INFO_STREAM(TCP.matrix());

    jointAnglesTar = jointAnglesAct;

    boundingBox = std::move(std::unique_ptr<BoundingBox>(new BoundingBox(nh_,TCP,RCM.translation(),boundingBoxSize,rcmDistance)));
    ros::Rate rate(rosRate);
    while(ros::ok())
    {
        ros::spinOnce();
        cycleTime = ros::Time::now().toSec() - lastTime;
        lastTime = ros::Time::now().toSec();
        std_msgs::Float64 timeMsg;
        timeMsg.data = cycleTime;
        cycleTimePub.publish(timeMsg);

        // Hier muss der TCP-Service gecallt werden
        masterslave::Manipulation manipulationService;
        Eigen::Affine3d TCP_old = TCP;
        tf::poseEigenToMsg(TCP,manipulationService.request.T_0_EE_old);
        tcpClient.call(manipulationService);
        tf::poseMsgToEigen(manipulationService.response.T_0_EE_new,TCP);

        ROS_INFO_STREAM(TCP.matrix());
        if(!TCP.isApprox(TCP_old))
        {
            masterslave::GeometricKinematicInverseKinematics inverseKinematicsService;
            tf::poseEigenToMsg(TCP, inverseKinematicsService.request.T_0_EE);
            inverseKinematicsClient.call(inverseKinematicsService);
            tf::poseMsgToEigen(inverseKinematicsService.response.T_0_FL,T_0_FL);
            stampedPose.pose = inverseKinematicsService.response.T_0_FL;
            lbrTargetPositionPub.publish(stampedPose);
            jointAnglesTar = Eigen::VectorXd::Map(inverseKinematicsService.response.jointAnglesTarget.data(),inverseKinematicsService.response.jointAnglesTarget.size());
            ROS_INFO_STREAM("jointAnglesTar: \n" << jointAnglesTar);
        }
        commandVelocities();
        rate.sleep();
    }
}


void GeometricKinematicCommander::Q4StateCallback(const sensor_msgs::JointStateConstPtr &state)
{
    jointAnglesAct(0) = state->position.at(0);
    callBacksCalled += 1 << 0;
}


void GeometricKinematicCommander::Q5StateCallback(const sensor_msgs::JointStateConstPtr &state)
{
    jointAnglesAct(1) = state->position.at(0);
    callBacksCalled += 1 << 1;
}

void GeometricKinematicCommander::Q6nStateCallback(const sensor_msgs::JointStateConstPtr &state)
{
    motorAngles(1) = state->position.at(0);
    // set  signal that callback Q6n was called
    Q6CallbacksCalled += 1 << 1;
    // check if both Q6 callbacks were called
    if(1 == Q6CallbacksCalled+1 >> 2)
    {
        calcQ6();
    }
}

void GeometricKinematicCommander::Q6pStateCallback(const sensor_msgs::JointStateConstPtr &state)
{
    motorAngles(0)= state->position.at(0);
    // set  signal that callback Q6n was called
    Q6CallbacksCalled += 1;
    // check if both Q6 callbacks were called
    if(1 == Q6CallbacksCalled+1 >> 2)
    {
        calcQ6();
    }
}

void GeometricKinematicCommander::buttonCallback(const masterslave::ButtonConstPtr &button)
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

void GeometricKinematicCommander::velocityCallback(const geometry_msgs::TwistStampedConstPtr &velocity)
{
    velocity_ = *velocity;
}

void GeometricKinematicCommander::calcQ6()
{
    jointAnglesAct(2) = (motorAngles(0) + motorAngles(1)) / 2;
    if(fabs(motorAngles(0) + motorAngles(1))<0)
        gripper_stop = true;
    else
        gripper_stop = false;
    // reset the callback counter
    Q6CallbacksCalled = 0;
    callBacksCalled += 1 << 2;
}

void GeometricKinematicCommander::commandVelocities()
{
    double gripperVelocity;
    std_msgs::Float64 Q4Vel, Q5Vel, Q6nVel, Q6pVel;
    Q4Vel.data = (jointAnglesTar(0) - jointAnglesAct(0));
    Q5Vel.data = (jointAnglesTar(1) - jointAnglesAct(1));
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

    Q6nVel.data = (jointAnglesTar(2)-jointAnglesAct(2));
    Q6pVel.data = (jointAnglesTar(2)-jointAnglesAct(2));
    // Stoppen der Greiferbacken, wenn eine der beiden am Anschlag ist, um Greiferöffnungswinkel nicht zu ändern
    if(motorAngles(1)>=0.95*M_PI && (jointAnglesTar(2)-jointAnglesAct(2))>0)
    {
        Q6nVel.data = 0;
    }
    if(motorAngles(0)<=-0.95*M_PI && (jointAnglesTar(2)-jointAnglesAct(2))<0)
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

int main(int argc, char** argv)
{
    ros::init(argc,argv,"GeometricKinematicCommander");
    ros::NodeHandle controlDeviceNH(argv[1]);
    ros::NodeHandle taskNH("dynamicReconfigure");
    GeometricKinematicCommander* commander= new GeometricKinematicCommander(controlDeviceNH, taskNH);

    return 0;
}



