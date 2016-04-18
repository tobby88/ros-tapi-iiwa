#include "commander/NumericKinematicCommander.h"


NumericKinematicCommander::NumericKinematicCommander(ros::NodeHandle& nh, ros::NodeHandle& drNH): nh_(nh)
{
    dynamic_reconfigure::Server<masterslave::MasterSlaveConfig> server(drNH);
    dynamic_reconfigure::Server<masterslave::MasterSlaveConfig>::CallbackType f;

    f = boost::bind(&NumericKinematicCommander::configurationCallback,this,_1,_2);

    server.setCallback(f);

    RCM = Eigen::Affine3d::Identity();
    motorAngles = Eigen::VectorXd::Zero(2);
    jointAnglesAct = Eigen::VectorXd::Zero(10);
    jointAnglesTar = Eigen::VectorXd::Zero(10);

    heightSafety = 0.05;

    cycleTimePub = nh_.advertise<std_msgs::Float64>("/cycleTime",1);
    getControlDevice();
    Q4StateSub = nh_.subscribe("/Q4/joint_states",1,&NumericKinematicCommander::Q4StateCallback, this);
    Q5StateSub = nh_.subscribe("/Q5/joint_states",1,&NumericKinematicCommander::Q5StateCallback, this);
    Q6nStateSub = nh_.subscribe("/Q6N/joint_states",1,&NumericKinematicCommander::Q6nStateCallback, this);
    Q6pStateSub = nh_.subscribe("/Q6P/joint_states",1,&NumericKinematicCommander::Q6pStateCallback, this);

    rcmClient = nh_.serviceClient<masterslave::NumericKinematicRCM>("/RCM");
    directKinematicsClient = nh_.serviceClient<masterslave::NumericKinematicDirectKinematics>("/directKinematics");
    inverseKinematicsClient = nh_.serviceClient<masterslave::NumericKinematicInverseKinematics>("/inverseKinematics");
    tcpClient = nh_.serviceClient<masterslave::Manipulation>("/Manipulation");
    stateService = nh_.serviceClient<masterslave::OpenIGTLStateService>("/openIGTLState");

    ros::Timer timer = nh_.createTimer(ros::Duration(0.02), &NumericKinematicCommander::statemachineThread, this);

    for(int i=0; i < 7; i++)
    {
       std::stringstream sstream;
       sstream << "/LBR/des/joint" << i+1;
       lbrJointAnglePub[i] = nh_.advertise<std_msgs::Float64>(sstream.str().c_str(),1);
       sstream.str(std::string());

       sstream << "/LBR/act/joint" << i+1;
       lbrJointAngleSub[i] = nh_.subscribe<sensor_msgs::JointState>(sstream.str().c_str(),1,boost::bind(&NumericKinematicCommander::lbrJointAngleCallback,this,_1,i));
       sstream.str(std::string());
    }

    Q4Pub = nh_.advertise<std_msgs::Float64>("/Q4/setPointVelocity",1);
    Q5Pub = nh_.advertise<std_msgs::Float64>("/Q5/setPointVelocity",1);
    Q6nPub = nh_.advertise<std_msgs::Float64>("/Q6N/setPointVelocity",1);
    Q6pPub = nh_.advertise<std_msgs::Float64>("/Q6P/setPointVelocity",1);

    ros::Rate waiteRate(25);
    while(ros::ok())
    {
        ros::spinOnce();
        if(state == MOVE_TO_POSE)
        {
           ROS_DEBUG("MOVE_TO_POSE");
           loop();
        }
        ROS_DEBUG("NOT_MOVE_TO_POSE");
    }
}

void NumericKinematicCommander::statemachineThread(const ros::TimerEvent& event)
{
    masterslave::OpenIGTLStateService stateStringMsg;
    //ROS_WARN_STREAM(state);
    switch(state)
    {
        case IDLE:
            if(stateService.exists()) stateStringMsg.request.state = "IDLE;";
            //ROS_INFO("IDLE");
            break;
        case FREE:
            if(stateService.exists())  stateStringMsg.request.state = "GravComp;";
            //ROS_INFO("FREE");
            break;
        case MOVE_TO_POSE:
            if(stateService.exists()) stateStringMsg.request.state = "MoveToPose;rob;";
            //ROS_INFO("MOVE_TO");
            break;
        default:
            ROS_ERROR("No valid state");
    }
    if(stateService.exists())
    {
        stateService.call(stateStringMsg);
        statemachineIsRunning = stateStringMsg.response.alive;
        ROS_DEBUG_STREAM("Service Call: alive: " << statemachineIsRunning);
    }
}


void NumericKinematicCommander::configurationCallback(masterslave::MasterSlaveConfig &config, uint32_t level)
{
    state = static_cast<OPENIGTL_STATE>(config.cur_state);
    rosRate = config.rosRate;
    setTrocar = config.set_Trocar;
    ROS_INFO("STATE CHANGED");
}

void NumericKinematicCommander::loop()
{
    ros::spinOnce();
    if(!((callBacksCalled+1) >> 10 >=1))
    {
        return;
    }

    double lastTime = ros::Time::now().toSec();

    ROS_INFO_STREAM(jointAnglesAct);
    masterslave::NumericKinematicRCM rcmService;
    if(setTrocar || RCM.isApprox(Eigen::Affine3d::Identity()))
    {
        std::vector<double> trocarAngles(jointAnglesAct.data(),jointAnglesAct.data()+jointAnglesAct.rows());
        rcmService.request.trocarAngles = trocarAngles;
        rcmClient.call(rcmService);
        tf::poseMsgToEigen(rcmService.response.trocar,RCM);
    }

    masterslave::NumericKinematicDirectKinematics directKinematicsService;
    std::vector<double> jointAnglesActual(jointAnglesAct.data(),jointAnglesAct.data()+jointAnglesAct.rows());
    directKinematicsService.request.jointAngles = jointAnglesActual;
    directKinematicsClient.call(directKinematicsService);
    directKinematicsService.request.jointAngles.clear();
    tf::poseMsgToEigen(directKinematicsService.response.T_0_EE,TCP);
    jointAnglesTar = jointAnglesAct;

    //boundingBox = std::move(std::unique_ptr<BoundingBox>(new BoundingBox(nh_,TCP,RCM.translation(),boundingBoxSize,rcmDistance)));

    ros::Rate rate(rosRate);

    while(ros::ok() && state == MOVE_TO_POSE)
    {
        ros::spinOnce();
        cycleTime = ros::Time::now().toSec() - lastTime;
        lastTime = ros::Time::now().toSec();
        std_msgs::Float64 timeMsg;
        timeMsg.data = cycleTime;
        cycleTimePub.publish(timeMsg);
        Eigen::Affine3d TCP_old = TCP;
        // Hier muss der TCP-Service gecallt werden
        masterslave::Manipulation manipulationService;
        tf::poseEigenToMsg(TCP,manipulationService.request.T_0_EE_old);

        tcpClient.call(manipulationService);
        tf::poseMsgToEigen(manipulationService.response.T_0_EE_new,TCP);
        if(!TCP.isApprox(TCP_old) /*&& boundingBox->checkBoundingBoxTCP(TCP)*/)
        {
            masterslave::NumericKinematicInverseKinematics inverseKinematicsService;
            tf::poseEigenToMsg(TCP,inverseKinematicsService.request.T_0_EE);
            ROS_DEBUG_STREAM(inverseKinematicsService.request.T_0_EE.position);

            if(inverseKinematicsClient.call(inverseKinematicsService))
            {
                jointAnglesTar = Eigen::VectorXd::Map(inverseKinematicsService.response.jointAnglesTarget.data(),inverseKinematicsService.response.jointAnglesTarget.size());
            }
            else
            {
                masterslave::NumericKinematicDirectKinematics directKinematicsService;
                std::vector<double> jointAnglesActual(jointAnglesAct.data(),jointAnglesAct.data()+jointAnglesAct.rows());
                directKinematicsService.request.jointAngles = jointAnglesActual;
                directKinematicsClient.call(directKinematicsService);
                directKinematicsService.request.jointAngles.clear();
                TCP = TCP_old;
            }
        }
        else
        {       
            masterslave::NumericKinematicDirectKinematics directKinematicsService;
            std::vector<double> jointAnglesActual(jointAnglesAct.data(),jointAnglesAct.data()+jointAnglesAct.rows());
            directKinematicsService.request.jointAngles = jointAnglesActual;
            directKinematicsClient.call(directKinematicsService);
            directKinematicsService.request.jointAngles.clear();
            tf::poseMsgToEigen(directKinematicsService.response.T_0_EE,TCP);

        }

        commandVelocities();
        for(int i=0;i<7;i++)
        {
            std_msgs::Float64 jointAngleTemp;
            jointAngleTemp.data = jointAnglesTar(i);
            lbrJointAnglePub[i].publish(jointAngleTemp);
        }
        rate.sleep();
    }
}

void NumericKinematicCommander::lbrJointAngleCallback(const sensor_msgs::JointStateConstPtr &state, int number)
{
    jointAnglesAct(number) = state->position[0];
    //check if all LBR callbacks are called once
    callBacksCalled += 1 << number;

}

/*void NumericKinematicCommander::calcQ6()
{
    if(motorAngles(0) - motorAngles(1)<0.0 && gripper_close)
    {
        gripper_stop = true;
    }
    else
    {
        gripper_stop = false;
        // reset the callback counter
    }
    Q6CallbacksCalled = 0;
    jointAnglesAct.tail(3)(2) = (motorAngles(0) + motorAngles(1)) / 2;
    callBacksCalled += 1 << 9;
    ROS_WARN_STREAM("gripper_stop: \n" << gripper_stop);
}*/

void NumericKinematicCommander::Q4StateCallback(const sensor_msgs::JointStateConstPtr &state)
{
    jointAnglesAct.tail(3)(0) = state->position.at(0);
    callBacksCalled += 1 << 7;
}


void NumericKinematicCommander::Q5StateCallback(const sensor_msgs::JointStateConstPtr &state)
{
    jointAnglesAct.tail(3)(1) = state->position.at(0);
    callBacksCalled += 1 << 8;
}

void NumericKinematicCommander::Q6nStateCallback(const sensor_msgs::JointStateConstPtr &state)
{
    motorAngles(0)= state->position.at(0);
    // set  signal that callback Q6n was called
    Q6CallbacksCalled += 1;
    // check if both Q6 callbacks were called
    if(1 == Q6CallbacksCalled+1 >> 2)
    {
        calcQ6();
        callBacksCalled += 1 << 9;
    }

}

void NumericKinematicCommander::Q6pStateCallback(const sensor_msgs::JointStateConstPtr &state)
{
    motorAngles(1) = state->position.at(0);
    // set  signal that callback Q6n was called
    Q6CallbacksCalled += 1 << 1;
    // check if both Q6 callbacks were called
    if(1 == Q6CallbacksCalled+1 >> 2)
    {
        calcQ6();
        callBacksCalled += 1 << 9;
    }
}

void NumericKinematicCommander::buttonCallback(const masterslave::ButtonConstPtr &button)
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

void NumericKinematicCommander::getControlDevice()
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
        buttonSub = nh_.subscribe(device_sstream.str().c_str(),10,&NumericKinematicCommander::buttonCallback, this);
        device_sstream.str(std::string());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"NumericKinematicCommander");
    ros::NodeHandle taskNH("dynamicReconfigure");
    ros::NodeHandle ControlDeviceNH(argv[1]);
    NumericKinematicCommander* commander = new NumericKinematicCommander(ControlDeviceNH, taskNH);

    return 0;
}

