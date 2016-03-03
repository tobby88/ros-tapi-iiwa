#include "task/ursulatask.h"

/**
 * @brief UrsulaTask::UrsulaTask
 * @param nh
 * @param rosRate
 */

UrsulaTask::UrsulaTask(ros::NodeHandle& nh, double rosRate): nh_(nh), rosRate_(rosRate)
{
    motorAngles = Eigen::VectorXd::Zero(2);
    jointAnglesAct = Eigen::VectorXd::Zero(10);
    jointAnglesTar = Eigen::VectorXd::Zero(10);

    heightSafety = 0.05;

    cycleTimePub = nh_.advertise<std_msgs::Float64>("/cycleTime",1);
    getControlDevice();
    Q4StateSub = nh_.subscribe("/Q4/joint_states",1,&UrsulaTask::Q4StateCallback, this);
    Q5StateSub = nh_.subscribe("/Q5/joint_states",1,&UrsulaTask::Q5StateCallback, this);
    Q6nStateSub = nh_.subscribe("/Q6N/joint_states",1,&UrsulaTask::Q6nStateCallback, this);
    Q6pStateSub = nh_.subscribe("/Q6P/joint_states",1,&UrsulaTask::Q6pStateCallback, this);
    lbrPositionSub = nh_.subscribe("/flangeLBR",1,&UrsulaTask::flangeCallback, this);

    rcmClient = nh_.serviceClient<masterslave::rcmTest>("/RCM");
    directKinematicsClient = nh_.serviceClient<masterslave::directKinematics>("/directKinematics");
    inverseKinematicsClient = nh_.serviceClient<masterslave::inverseKinematics>("/inverseKinematics");

    for(int i=0; i < 7; i++)
    {
       std::stringstream sstream;
       sstream << "/LBR/des/joint" << i+1;
       lbrJointAnglePub[i] = nh_.advertise<std_msgs::Float64>(sstream.str().c_str(),1);
       sstream.str(std::string());

       sstream << "/LBR/act/joint" << i+1;
       lbrJointAngleSub[i] = nh_.subscribe<sensor_msgs::JointState>(sstream.str().c_str(),1,boost::bind(&UrsulaTask::lbrJointAngleCallback,this,_1,i));
       sstream.str(std::string());
    }

    Q4Pub = nh_.advertise<std_msgs::Float64>("/Q4/setPointVelocity",1);
    Q5Pub = nh_.advertise<std_msgs::Float64>("/Q5/setPointVelocity",1);
    Q6nPub = nh_.advertise<std_msgs::Float64>("/Q6N/setPointVelocity",1);
    Q6pPub = nh_.advertise<std_msgs::Float64>("/Q6P/setPointVelocity",1);


    ros::Rate waiteRate(0.5);
    while(ros::ok() && lbrPositionSub.getNumPublishers() !=1)
    {
        ros::spinOnce();
        waiteRate.sleep();
    }

    loop();
}

void UrsulaTask::loop()
{
    ros::spinOnce();
    double lastTime = ros::Time::now().toSec();

    masterslave::rcmTest rcmService;
    for(int i=0; i<jointAnglesAct.rows();i++)
    {
        rcmService.request.trocarAngles.push_back(jointAnglesAct[i]);
    }
    rcmClient.call(rcmService);

    rcmService.request.trocarAngles.clear();
    tf::poseMsgToEigen(rcmService.response.trocar,RCM);

    masterslave::directKinematics directKinematicsService;

    for(int i=0; i<jointAnglesAct.rows();i++)
    {
        directKinematicsService.request.jointAngles.push_back(jointAnglesAct[i]);
    }

    directKinematicsClient.call(directKinematicsService);
    directKinematicsService.request.jointAngles.clear();
    tf::poseMsgToEigen(directKinematicsService.response.T_0_EE,TCP);
    jointAnglesTar = jointAnglesAct;
    ros::Rate rate(1000);
    while(ros::ok())
    {
        ros::spinOnce();
        cycleTime = ros::Time::now().toSec() - lastTime;
        lastTime = ros::Time::now().toSec();
        std_msgs::Float64 timeMsg;
        timeMsg.data = cycleTime;
        cycleTimePub.publish(timeMsg);
        //ROS_INFO_STREAM("cycleTime: " << cycleTime);
        Eigen::Affine3d TCP_old = TCP;
        TCP = moveEEFrame(TCP);

        if(!TCP.isApprox(TCP_old))
        {
            masterslave::inverseKinematics inverseKinematicsService;
            tf::poseEigenToMsg(TCP,inverseKinematicsService.request.T_0_EE);
            ROS_DEBUG_STREAM(inverseKinematicsService.request.T_0_EE.position);

            inverseKinematicsClient.call(inverseKinematicsService);
            for(int i=0;i<10;i++)
            {
                jointAnglesTar[i] = inverseKinematicsService.response.jointAnglesTarget.at(i);
            }

        }
        commandVelocities();
        /*if(newJointAnglesArrived == (1 << 10)-1)
        {
             ROS_INFO("new JointAngles");
             std::vector<double> jointAngles(jointAnglesAct.data(),jointAnglesAct.data()+jointAnglesAct.rows());
             masterslave::directKinematics directKinematicsService;

             for(int i=0; i<jointAnglesAct.rows();i++)
             {
                 directKinematicsService.request.jointAngles.push_back(jointAnglesAct[i]);
             }

             directKinematicsClient.call(directKinematicsService);
             directKinematicsService.request.jointAngles.clear();
        }*/
        newJointAnglesArrived = 0;
        rate.sleep();
    }
    ros::shutdown();
}


void UrsulaTask::flangeCallback(const geometry_msgs::PoseStampedConstPtr& flangePose)
{
    tf::poseMsgToEigen(flangePose->pose,startPositionLBR);
    lbrPositionSub.shutdown();
}

void UrsulaTask::lbrJointAngleCallback(const sensor_msgs::JointStateConstPtr &state, int number)
{
    jointAnglesAct(number) = state->position[0];
    newJointAnglesArrived += 1 << number;
}

void UrsulaTask::calcQ6()
{
    jointAnglesAct.tail(3)(2) = (motorAngles(0) + motorAngles(1)) / 2;
    if(fabs(motorAngles(0) + motorAngles(1))<0)
        gripper_stop = true;
    else
        gripper_stop = false;
}

void UrsulaTask::Q4StateCallback(const sensor_msgs::JointStateConstPtr &state)
{
    jointAnglesAct.tail(3)(0) = state->position.at(0);
    newJointAnglesArrived += 1 << 7;
}


void UrsulaTask::Q5StateCallback(const sensor_msgs::JointStateConstPtr &state)
{
    jointAnglesAct.tail(3)(1) = state->position.at(0);
    newJointAnglesArrived += 1 << 8;
}

void UrsulaTask::Q6nStateCallback(const sensor_msgs::JointStateConstPtr &state)
{
    motorAngles(0)= state->position.at(0);
    newJointAnglesArrived += 1 << 9;
}

void UrsulaTask::Q6pStateCallback(const sensor_msgs::JointStateConstPtr &state)
{
    motorAngles(1) = state->position.at(0);
    calcQ6();
}

void UrsulaTask::velocityCallback(const geometry_msgs::TwistStampedConstPtr &velocity)
{
    velocity_ = *velocity;
}

void UrsulaTask::buttonCallback(const masterslave::ButtonConstPtr &button)
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

void UrsulaTask::getControlDevice()
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
        device_sstream << it->first << "/Velocity";
        velocitySub = nh_.subscribe(device_sstream.str().c_str(),10,&UrsulaTask::velocityCallback,this);
        device_sstream.str(std::string());
        device_sstream << it->first << "/Buttons";
        buttonSub = nh_.subscribe(device_sstream.str().c_str(),10,&UrsulaTask::buttonCallback, this);
        device_sstream.str(std::string());
    }
}

Eigen::Affine3d UrsulaTask::moveEEFrame(Eigen::Affine3d oldFrame)
{
    Eigen::Affine3d newFrame;
    newFrame.setIdentity();
    newFrame.translate(oldFrame.translation());



    /*if(aperture<=apertureLimit*DEG_TO_RAD || (velocity_.twist.linear.x/cos(polarAngle)<0 || velocity_.twist.linear.y/sin(polarAngle)<0))
    {
        newFrame.translate(Eigen::Vector3d(velocity_.twist.linear.x*cycleTime,velocity_.twist.linear.y*cycleTime,0));
    }*/

    newFrame.translate(Eigen::Vector3d(velocity_.twist.linear.x*cycleTime,velocity_.twist.linear.y*cycleTime,0));

    //ROS_INFO_STREAM(oldFrame.matrix());

    newFrame.translate(Eigen::Vector3d(0,0,velocity_.twist.linear.z*cycleTime));


    if((oldFrame.translation().z()+heightSafety<RCM.translation().z() || velocity_.twist.linear.z > 0) && (oldFrame.translation().z() > heightSafety || velocity_.twist.linear.z < 0))
    {

    }


    newFrame.rotate(QuaternionFromEuler(Eigen::Vector3d(velocity_.twist.angular.x*cycleTime,velocity_.twist.angular.y*cycleTime,velocity_.twist.angular.z*cycleTime),true));


    newFrame.rotate(oldFrame.rotation());

    //Plausibilitätskontrolle*/
    return newFrame;
}

void UrsulaTask::commandVelocities()
{
    double gripperVelocity;
    std_msgs::Float64 Q4Vel, Q5Vel, Q6nVel, Q6pVel;
    Q4Vel.data = (jointAnglesTar(7) - jointAnglesAct(7));
    Q5Vel.data = (jointAnglesTar(8) - jointAnglesAct(8));
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

    Q6nVel.data = (jointAnglesTar(9)-jointAnglesAct(9));
    Q6pVel.data = (jointAnglesTar(9)-jointAnglesAct(9));
    // Stoppen der Greiferbacken, wenn eine der beiden am Anschlag ist, um Greiferöffnungswinkel nicht zu ändern
    if(motorAngles(1)>=0.95*M_PI && (jointAnglesTar(9)-jointAnglesAct(9))>0)
    {
        Q6nVel.data = 0;
    }
    if(motorAngles(0)<=-0.95*M_PI && (jointAnglesTar(9)-jointAnglesAct(9))<0)
    {
        Q6pVel.data = 0;
    }
    Q6nVel.data += gripperVelocity;
    Q6pVel.data -= gripperVelocity;

    Q4Pub.publish(Q4Vel);
    Q5Pub.publish(Q5Vel);
    Q6nPub.publish(Q6nVel);
    Q6pPub.publish(Q6pVel);

    for(int i=0;i<7;i++)
    {
        std_msgs::Float64 jointAngleTemp;
        jointAngleTemp.data = jointAnglesTar(i);
        lbrJointAnglePub[i].publish(jointAngleTemp);
    }

}

