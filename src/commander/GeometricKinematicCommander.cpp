#include "masterslave/commander/GeometricKinematicCommander.h"

GeometricKinematicCommander::GeometricKinematicCommander(ros::NodeHandle &nh, ros::NodeHandle& drNH): nh_(nh)
{    
    dynamic_reconfigure::Server<masterslave::MasterSlaveConfig> server(drNH);
    dynamic_reconfigure::Server<masterslave::MasterSlaveConfig>::CallbackType f;

    jointAnglesAct = Eigen::Vector3d::Zero(3);
    jointAnglesTar = Eigen::Vector3d::Zero(3);
    motorAngles = Eigen::Vector2d::Zero();

    f = boost::bind(&ICommander::configurationCallback,this,_1,_2);

    RCM = Eigen::Affine3d::Identity();

    server.setCallback(f);

    getControlDevice();
    Q4StateSub = nh_.subscribe("/Q4/joint_states",1,&GeometricKinematicCommander::Q4StateCallback, this);
    Q5StateSub = nh_.subscribe("/Q5/joint_states",1,&GeometricKinematicCommander::Q5StateCallback, this);
    Q6nStateSub = nh_.subscribe("/Q6N/joint_states",1,&GeometricKinematicCommander::Q6nStateCallback, this);
    Q6pStateSub = nh_.subscribe("/Q6P/joint_states",1,&GeometricKinematicCommander::Q6pStateCallback, this);
    lbrPositionSub = nh_.subscribe("/flangeLBR",1,&GeometricKinematicCommander::flangeCallback, this);
    pliersDistanceSub = nh_.subscribe("/pliersDistance",1,&GeometricKinematicCommander::pliersDistanceCallback,this);


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
    ROS_INFO_STREAM(nh_.getNamespace() << " has found the start position of the robot!");
    setZero();
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
    switch(state)
    {
        case IDLE:
            stateStringMsg.request.state.append("IDLE;");
            ROS_INFO("IDLE");
            break;
        case FREE:
            stateStringMsg.request.state.append("GravComp;");
            ROS_INFO("FREE");
            break;
        case MOVE_TO_POSE:
            stateStringMsg.request.state.append("MoveToPose;rob;");
            ROS_INFO("MOVE_TO");
            break;
        default:
            ROS_ERROR("No Valid state");
    }
    if(stateService.exists())
    {
        stateService.call(stateStringMsg);
        statemachineIsRunning = stateStringMsg.response.alive;
        ROS_INFO_STREAM("Service Call: alive: " << statemachineIsRunning);
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
        // Keine Möglichkeit das in ICommander zu schieben, da hier ein Callback definiert wird
        buttonSub = nh_.subscribe(device_sstream.str().c_str(),10,&GeometricKinematicCommander::buttonCallback, this);
        device_sstream.str(std::string());
    }
}

void GeometricKinematicCommander::flangeCallback(const geometry_msgs::PoseStampedConstPtr& flangePose)
{
    tf::poseMsgToEigen(flangePose->pose,startPositionLBR);
    if(state == MOVE_TO_POSE)
    {
        callBacksCalled += 1 << 3;
    }
    else
    {
        callBacksCalled = 0;
    }
}

void GeometricKinematicCommander::loop()
{
    if(callBacksCalled < 8) return;
    double lastTime = ros::Time::now().toSec();
    masterslave::GeometricKinematicRCM rcmService;
    geometry_msgs::PoseStamped stampedPose;
    
    //Testen, ob der Trokar schon gesetzt ist oder ob sich das System im Trokarsetzmodus befindet
    if(setTrocar || RCM.isApprox(Eigen::Affine3d::Identity()))
    {
        ROS_INFO_STREAM(startPositionLBR.matrix());
        tf::poseEigenToMsg(startPositionLBR,rcmService.request.T_0_FL);
        rcmClient.call(rcmService);
    }

    boundingBox = std::move(std::unique_ptr<BoundingBox>(new BoundingBox(nh_,TCP,RCM.translation(),boundingBoxSize,rcmDistance)));

    masterslave::GeometricKinematicDirectKinematics directKinematicsService;
    std::vector<double> jointAngles(jointAnglesAct.data(),jointAnglesAct.data()+jointAnglesAct.rows());
    directKinematicsService.request.jointAngles = jointAngles;
    tf::poseEigenToMsg(startPositionLBR,directKinematicsService.request.T_0_FL);
    directKinematicsClient.call(directKinematicsService);
    tf::poseMsgToEigen(directKinematicsService.response.T_0_EE,TCP);
    ROS_INFO_STREAM(TCP.matrix());

    jointAnglesTar = jointAnglesAct;

    //boundingBox = std::move(std::unique_ptr<BoundingBox>(new BoundingBox(nh_,TCP,RCM.translation(),boundingBoxSize,rcmDistance)));
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

        ROS_DEBUG_STREAM(TCP.matrix());
        //Nur Änderung der Gelenkwinkel, wenn sich der TCP geändert hat
        if(!TCP.isApprox(TCP_old))
        {
            // Inverser Kinematik-Service call
            masterslave::GeometricKinematicInverseKinematics inverseKinematicsService;
            tf::poseEigenToMsg(TCP, inverseKinematicsService.request.T_0_EE);
            inverseKinematicsClient.call(inverseKinematicsService);
            tf::poseMsgToEigen(inverseKinematicsService.response.T_0_FL,T_0_FL);
            stampedPose.pose = inverseKinematicsService.response.T_0_FL;
            //Einlesen der Gelenkwinkel von std::vector auf Eigen::VectorXd
            jointAnglesTar = Eigen::VectorXd::Map(inverseKinematicsService.response.jointAnglesTarget.data(),inverseKinematicsService.response.jointAnglesTarget.size());
            ROS_DEBUG_STREAM("jointAnglesTar: \n" << jointAnglesTar);
            lbrTargetPositionPub.publish(stampedPose);
        }
        else if(!boundingBox->checkBoundingBoxTCP(TCP))
        {
            TCP = TCP_old;
        }
        commandVelocities();
        rate.sleep();
    }
}


void GeometricKinematicCommander::Q4StateCallback(const sensor_msgs::JointStateConstPtr &state)
{
    jointAnglesAct(0) = state->position.at(0);
    //callBacksCalled += 1 << 0;
}


void GeometricKinematicCommander::Q5StateCallback(const sensor_msgs::JointStateConstPtr &state)
{
    jointAnglesAct(1) = state->position.at(0);
    //callBacksCalled += 1 << 1;
}

void GeometricKinematicCommander::Q6nStateCallback(const sensor_msgs::JointStateConstPtr &state)
{
    motorAngles(0) = state->position.at(0);
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
    motorAngles(1)= state->position.at(0);
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

void GeometricKinematicCommander::pliersDistanceCallback(const std_msgs::Float64ConstPtr &value)
{
    // Öffnungswinkel berechnen mit Sinus von der halben Öffnungsdistanz durch die Zangenlänge
    pliersOpeningAngle = sin((value->data-PLIERS_DISTANCE_TOLERANCE)/(2*PLIERS_LENGTH));
    if(pliersOpeningAngle>M_PI/4)
    {
        pliersOpeningAngle = M_PI/4;
    }
    else if(pliersOpeningAngle<0)
    {
        pliersOpeningAngle = 0;
    }
}




int main(int argc, char** argv)
{
    ros::init(argc,argv,"GeometricKinematicCommander");
    ros::NodeHandle controlDeviceNH(argv[1]);
    ros::NodeHandle taskNH("dynamicReconfigure");
    GeometricKinematicCommander* commander= new GeometricKinematicCommander(controlDeviceNH, taskNH);

    return 0;
}



