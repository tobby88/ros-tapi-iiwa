#include "masterslave/rosopenigtlbridge.h"
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
    dynamic_reconfigure::Server<masterslave::rosigtlbridgeConfig> serverIGTL;
    dynamic_reconfigure::Server<masterslave::rosigtlbridgeConfig>::CallbackType cbIGTL;

    cbIGTL = boost::bind(&RosOpenIgtlBridge::configurationIGTLCallback,this,_1,_2);

    serverIGTL.setCallback(cbIGTL);

    this->flangeTargetSub = nh_.subscribe("/flangeTarget",1,&RosOpenIgtlBridge::transformCallback,this);
    this->flangePub = nh_.advertise<geometry_msgs::PoseStamped>("/flangeLBR",1);
    sendTransformFlag = false;
    start_ = false;
    stop_ = false;
    CMD_UID=0;
    sampleTime_  = 0.05;
    commandPort_ = 49001;
    transformPort_ = 49002;
    commandIP_ = "172.31.1.147";
    transformIP_ = "172.31.1.147";
    transformReceived_ = false;

    boost::thread(boost::bind(&RosOpenIgtlBridge::openIGTLinkTransformThread,this));
    openIGTLinkThread();
}

void RosOpenIgtlBridge::transformCallback(geometry_msgs::PoseStampedConstPtr transform)
{
    std::stringstream sstream;
    std::vector<double> transformVector;

    // hier wird der OpenIGTLink-String zusammengestellt
    sstream << "MoveToPose;" << "rob;";
    transformVector = this->rosPoseToIGTL(transform->pose);
    for(std::vector<double>::iterator it=transformVector.begin(); it!=transformVector.end();it++)
    {
        //Hinzufügen einer Koordinate der Transformationsmatrix
        sstream << *it << ";";
    }
    this->poseFL_new = transform->pose;
    openIGTLCommandString = sstream.str();
    sstream.str(std::string());
    transformReceived_ = true;
}

//second thread to get transform messages from the OpenIGTL-Interface
void RosOpenIgtlBridge::openIGTLinkTransformThread()
{
    ROS_INFO("Entering Transformation Thread");
    transformSocket_ = igtl::ClientSocket::New();
    rTransform = transformSocket_->ConnectToServer(transformIP_.c_str(),transformPort_);
    if(rTransform==-1)
    {
        ROS_ERROR("OpenIGTLink Transform Interface is not available");
        exit(-1);
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
    rCommand = commandSocket_->ConnectToServer(commandIP_.c_str(),commandPort_);

    ROS_DEBUG_STREAM("rCommand: " << rCommand << " rTransform: " << rTransform);
    if(rCommand==-1)
    {
        ROS_ERROR("OpenIGTLink Interface is not available");
        exit(-1);
    }
    this->sendIdleState(commandSocket_);
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
            this->sendTransform(commandSocket_);
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

int RosOpenIgtlBridge::sendIdleState(igtl::ClientSocket::Pointer &socket)
{
    std::stringstream sstream;
    igtl::StringMessage::Pointer commandString;
    CMD_UID++;
    sstream << "CMD_" << CMD_UID;
    commandString = igtl::StringMessage::New();
    commandString->SetDeviceName(sstream.str().c_str());
    commandString->SetString("Idle;");
    commandString->Pack();
    ROS_INFO("Send Idle State");
    return socket->Send(commandString->GetPackPointer(),commandString->GetPackSize());
}

int RosOpenIgtlBridge::sendTransform(igtl::ClientSocket::Pointer &socket)
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

    ROS_DEBUG_STREAM("Send Transform" << " string: " << openIGTLCommandString);

    transformStringMsg->SetString(openIGTLCommandString);
    transformStream.str(std::string());
    transformStringMsg->Pack();
    return socket->Send(transformStringMsg->GetPackPointer(),transformStringMsg->GetPackSize());
}

std::vector<double> RosOpenIgtlBridge::rosPoseToIGTL(geometry_msgs::Pose pose)
{
    pose.position.x*=MM_TO_M;
    pose.position.y*=MM_TO_M;
    pose.position.z*=MM_TO_M;
    Eigen::Affine3d poseToEigen;
    tf::poseMsgToEigen(pose,poseToEigen);
    std::vector<double> returnValue;
    returnValue.push_back(poseToEigen.translation().x());
    returnValue.push_back(poseToEigen.translation().y());
    returnValue.push_back(poseToEigen.translation().z());
    for(int row=0;row<3;row++)
    {
        for(int col=0; col<3; col++)
        {
            returnValue.push_back(poseToEigen.matrix()(row,col));
        }
    }
    return returnValue;

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

void RosOpenIgtlBridge::configurationIGTLCallback(masterslave::rosigtlbridgeConfig &config, uint32_t level)
{
    commandIP_ = config.controlIP;
    transformIP_ = config.transformIP;

    commandPort_ = config.controlPort;
    transformPort_ = config.transformPort;

}

int main(int argc, char** argv)
{
    ros::init(argc,argv, "RosOpenIGTLBridge");
    ros::NodeHandle RosOpenIGTLBridgeHandle;

    RosOpenIgtlBridge bridge(RosOpenIGTLBridgeHandle);
    return 0;
}
