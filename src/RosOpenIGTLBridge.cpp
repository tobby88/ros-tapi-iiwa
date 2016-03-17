#include "masterslave/RosOpenIGTLBridge.h"
#include "ros/ros.h"

/*
 * @file rosopenigtlbridge.cpp
 *
 * @author Fabian Baier
 *
 */

#define M_TO_MM 1.0/1000.0f

#define MM_TO_M 1000

/*
 * @class ROS-OpenIGTLink-Bridge-Klasse
 *
 * @brief Diese Klasse sorgt zur Anbindung des OpenIGTLink-Protokolls an ROS für das URSULA-Projekt.
 *
 * Mithilfe dieser Klasse wird der LBR iiwa des URSULA-Versuchsstandes an das URSULA-System angebunden.
 * Hier wird die aktuelle Roboterflanschlage von der Robotersteuerung übermittelt und an ROS übergeben.
 * Die neuberechnete Sollflanschlage des LBR wird über ein ROS-Topic empfangen und über OpenIGTLink versendet.
 */
RosOpenIgtlBridge::RosOpenIgtlBridge(ros::NodeHandle nh): nh_(nh)
{
    flangeTargetSub = nh_.subscribe("/flangeTarget",1,&RosOpenIgtlBridge::transformCallback,this);

    flangePub = nh_.advertise<geometry_msgs::PoseStamped>("/flangeLBR",1);

    for(int i=0; i < 7; i++)
    {
       std::stringstream sstream;
       sstream << "/LBR/des/joint" << i+1;
       lbrJointAngleSub[i] = nh_.subscribe<std_msgs::Float64>(sstream.str().c_str(),1,boost::bind(&RosOpenIgtlBridge::lbrJointAngleCallback,this,_1,i));
       sstream.str(std::string());

       sstream << "/LBR/act/joint" << i+1;
       lbrJointAnglePub[i] = nh_.advertise<sensor_msgs::JointState>(sstream.str().c_str(),1);
       sstream.str(std::string());
    }

    stateServiceServer = nh_.advertiseService("/openIGTLState",&RosOpenIgtlBridge::stateService,this);
    boost::thread(boost::bind(&RosOpenIgtlBridge::openIGTLinkTransformThread,this));
    openIGTLinkThread();
}

void RosOpenIgtlBridge::transformCallback(geometry_msgs::PoseStampedConstPtr transform)
{
    poseFL_new = transform->pose;
    rosTransformReceived_ = true;
}

//second thread to get transform messages from the OpenIGTL-Interface
void RosOpenIgtlBridge::openIGTLinkTransformThread()
{
    ROS_INFO("Entering Transformation Thread");
    transformSocket_ = igtl::ClientSocket::New();
    transformSocket_->DebugOff();

    double startTime = ros::Time::now().toSec();
    double endTime = ros::Time::now().toSec();
    do
    {
        rTransform = transformSocket_->ConnectToServer(TRANSFORM_IP,TRANSFORM_PORT);
        endTime = ros::Time::now().toSec();
        ROS_WARN_NAMED("ROSOpenIGTL","Trying to connect to OpenIGTL-Server");
    }
    while(rTransform==-1 && ros::ok() && endTime - startTime <= CONNECTION_TIMEOUT);
    if(rTransform==-1)
    {
        ROS_ERROR("No OpenIGTLink-Server available");
        ros::shutdown();
    }

    while(rTransform!=-1 && ros::ok())
    {
        igtl::MessageHeader::Pointer messageHeaderTransform;
        messageHeaderTransform = igtl::MessageHeader::New();
        this->receiveTransform(transformSocket_,messageHeaderTransform);
        rTransform = transformSocket_->GetConnected();
    }
    transformSocket_->CloseSocket();
}

void RosOpenIgtlBridge::openIGTLinkThread()
{
    ROS_INFO("Entering Command Thread");
    commandSocket_ = igtl::ClientSocket::New();
    commandSocket_->DebugOff();
    rCommand=-1;
    double startTime = ros::Time::now().toSec();
    double endTime = ros::Time::now().toSec();
    ROS_INFO_STREAM("rCommand: " << rCommand);
    do
    {
        rCommand = commandSocket_->ConnectToServer(COMMAND_IP,COMMAND_PORT);
        endTime = ros::Time::now().toSec();
        ROS_WARN_NAMED("ROSOpenIGTL","Trying to connect to OpenIGTL-Server");
    }
    while(rCommand==-1 && ros::ok() && endTime - startTime <= CONNECTION_TIMEOUT);
    if(rCommand==-1)
    {
        ROS_ERROR("No OpenIGTLink-Server available");
        ros::shutdown();
    }

    ROS_DEBUG_STREAM("rCommand: " << rCommand << " rTransform: " << rTransform);

    this->sendCommand(commandSocket_,"Idle;");
    ros::Rate rate(1/sampleTime_);
    while(rCommand!=-1 && ros::ok())
    {
        ros::spinOnce();
        if(sendTransformFlag)
        {
            sendTransformFlag = false;
            igtl::MessageHeader::Pointer messageHeaderCommand;
            messageHeaderCommand = igtl::MessageHeader::New();
            positionReached_ = this->positionReached(commandSocket_,messageHeaderCommand);
        }
        // Get pose of flange
        this->transformUpdateMutex_.lock();
            geometry_msgs::PoseStamped poseFLmsg;
            poseFLmsg.header.stamp = ros::Time::now();
            poseFLmsg.pose = poseFL;
            this->flangePub.publish(poseFLmsg);
        this->transformUpdateMutex_.unlock();
        if(transformReceived_)
        {
            this->sendCommand(commandSocket_,openIGTLCommandString);
            transformReceived_ = false;
        }
        rCommand = commandSocket_->GetConnected();
        rate.sleep();
    }
    commandSocket_->CloseSocket();
    stop_ = true;
    ros::shutdown();

}

int RosOpenIgtlBridge::positionReached(igtl::ClientSocket::Pointer &socket, igtl::MessageBase::Pointer &msgHeader)
{
    msgHeader->InitPack();
    rCommand = socket->Receive(msgHeader->GetPackPointer(),msgHeader->GetPackSize());
    msgHeader->Unpack();
    igtl::StringMessage::Pointer stringMsg;
    stringMsg->SetMessageHeader(msgHeader);
    stringMsg->AllocatePack();
    socket->Receive(stringMsg->GetPackBodyPointer(), stringMsg->GetPackBodySize());
    stringMsg->Unpack();
    std::string msg = stringMsg->GetString();
    if(msg == "true")
    {
        return 1;
    }
    return 0;
}

int RosOpenIgtlBridge::receiveTransform(igtl::ClientSocket::Pointer &socket, igtl::MessageBase::Pointer &msgHeader)
{
    igtl::TransformMessage::Pointer transformMsg;
    msgHeader->InitPack();
    rTransform = socket->Receive(msgHeader->GetPackPointer(),msgHeader->GetPackSize());
    msgHeader->Unpack();
    transformMsg = igtl::TransformMessage::New();
    transformMsg->SetMessageHeader(msgHeader);
    transformMsg->AllocatePack();
    socket->Receive(transformMsg->GetPackBodyPointer(), transformMsg->GetPackBodySize());
    transformMsg->Unpack();
    if(strcmp(transformMsg->GetDeviceName(),"T_EE")==0)
    {
        this->transformUpdateMutex_.lock();
        transformMsg->GetMatrix(T_FL);
        poseFL = this->igtlMatrixToRosPose(T_FL);
        this->transformUpdateMutex_.unlock();
        return 1;
    }
    return 0;
}


int RosOpenIgtlBridge::sendCommand(igtl::ClientSocket::Pointer &socket, std::string command)
{
    igtl::StringMessage::Pointer transformStringMsg;
    transformStringMsg = igtl::StringMessage::New();
    std::stringstream transformStream;
    CMD_UID++;
    if(this->CMD_UID > 999999999999) this->CMD_UID =0;
    transformStream << "CMD_" << CMD_UID;
    // Set UID
    transformStringMsg->SetDeviceName(transformStream.str().c_str());
    transformStream.str(std::string());
    transformStringMsg->SetString(command);
    transformStringMsg->Pack();
    return socket->Send(transformStringMsg->GetPackPointer(),transformStringMsg->GetPackSize());
}

std::string RosOpenIgtlBridge::rosPoseToIGTL(geometry_msgs::Pose pose)
{
    std::stringstream sstream;
    pose.position.x*=MM_TO_M;
    pose.position.y*=MM_TO_M;
    pose.position.z*=MM_TO_M;
    Eigen::Affine3d poseToEigen;
    tf::poseMsgToEigen(pose,poseToEigen);

    sstream << poseToEigen.translation().x();
    sstream << poseToEigen.translation().y();
    sstream << poseToEigen.translation().z();
    for(int row=0;row<3;row++)
    {
        for(int col=0; col<3; col++)
        {
            sstream << poseToEigen.matrix()(row,col);
        }
    }
    return sstream.str();

}

geometry_msgs::Pose RosOpenIgtlBridge::igtlMatrixToRosPose(igtl::Matrix4x4& igtlMatrix)
{
    geometry_msgs::Pose returnValue;
    Eigen::Affine3d eigenMatrix;
    for(int row=0;row<4;row++)
    {
        for(int col=0;col<4;col++)
        {
            eigenMatrix.matrix()(row,col) = igtlMatrix[row][col];
        }
    }
    tf::poseEigenToMsg(eigenMatrix,returnValue);
    returnValue.position.x *= M_TO_MM;
    returnValue.position.y *= M_TO_MM;
    returnValue.position.z *= M_TO_MM;
    return returnValue;
}

bool RosOpenIgtlBridge::stateService(masterslave::OpenIGTLStateService::Request &req, masterslave::OpenIGTLStateService::Response &res)
{
    std::stringstream sstream;
    if(rTransform==-1)
    {
        res.alive = false;
    }
    else
    {
        res.alive = true;
    }
    stateString = req.state;
    sstream << stateString;
    if(rosTransformReceived_ && stateString == "MoveToPose;rob;")
    {
        sstream << rosPoseToIGTL(poseFL_new);
        rosTransformReceived_ = false;
    }
    else if(stateString == "MoveToJointAngles;")
    {
        for(int i=0;i<jointAngles_new.rows();i++)
        {
            sstream << jointAngles_new(i) << ";";
        }
    }
    openIGTLCommandString = sstream.str();
    ROS_INFO_STREAM("Commandstring: " << openIGTLCommandString);
    sstream.str(std::string());
    return true;
}

void RosOpenIgtlBridge::lbrJointAngleCallback(const std_msgs::Float64ConstPtr &jointAngle, int number)
{
    jointAngles_new(number) = jointAngle->data;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv, "RosOpenIGTLBridge");
    ros::NodeHandle RosOpenIGTLBridgeHandle;

    RosOpenIgtlBridge bridge(RosOpenIGTLBridgeHandle);
    return 0;
}
