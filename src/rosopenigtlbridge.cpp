#include "masterslave/rosopenigtlbridge.h"
#include "ros/ros.h"

RosOpenIgtlBridge::RosOpenIgtlBridge(ros::NodeHandle nh): nh_(nh)
{
    nh_.subscribe("/flangeTarget",1,&RosOpenIgtlBridge::transformCallback,this);
    nh_.advertise<geometry_msgs::PoseStamped>("/flangeLBR",1);
    sendTransformFlag = false;
    CMD_UID=0;
    sampleTime_  = 0.02;
    commandPort_ = 49001;
    transformPort_ = 49002;
    commandIP_ = "172.31.1.147";
    transformIP_ = "172.31.1.147";

    openIGTLinkThread();

}

void RosOpenIgtlBridge::transformCallback(geometry_msgs::PoseStampedConstPtr transform)
{
    std::stringstream sstream;
    std::vector<double> transformVector;
    sstream << "MoveToPose;" << "rob;";
    transformVector = this->rosPoseToIGTL(transform->pose);
    for(std::vector<double>::iterator it=transformVector.begin(); it!=transformVector.end();it++)
    {
        sstream << *it << ";";
    }
    openIGTLCommandString = sstream.str();
    sstream.str(std::string());
}

void RosOpenIgtlBridge::openIGTLinkThread()
{
    commandSocket_ = igtl::ClientSocket::New();
    transformSocket_ = igtl::ClientSocket::New();
    rCommand = commandSocket_->ConnectToServer(commandIP_.c_str(),commandPort_);
    rTransform = transformSocket_->ConnectToServer(transformIP_.c_str(),transformPort_);
    this->sendIdleState(commandSocket_);
    ros::Rate rate(1/sampleTime_);
    while(rCommand!=0 && rTransform!=0 && ros::ok())
    {
        ros::spinOnce();
        if(sendTransformFlag)
        {
            igtl::MessageHeader::Pointer messageHeaderCommand;
            messageHeaderCommand = igtl::MessageHeader::New();
            this->positionReached(commandSocket_,messageHeaderCommand);

            igtl::MessageHeader::Pointer messageHeaderTransform;
            messageHeaderTransform = igtl::MessageHeader::New();
            this->receiveTransform(transformSocket_,messageHeaderTransform);
        }

        this->sendTransformFlag = this->sendTransform(commandSocket_);
        rate.sleep();

    }


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
    if(transformMsg->GetDeviceName() == "T_EE")
    {
        transformMsg->GetMatrix(T_FL);
        poseFL = this->igtlMatrixToRosPose(T_FL);
        return 1;
    }
    return 0;
}

int RosOpenIgtlBridge::sendIdleState(igtl::ClientSocket::Pointer &socket)
{
    igtl::StringMessage::Pointer commandString;
    commandString = igtl::StringMessage::New();
    commandString->SetString("Idle;");
    commandString->Pack();
    CMD_UID++;
    return socket->Send(commandString->GetPackPointer(),commandString->GetPackSize());
}

int RosOpenIgtlBridge::sendTransform(igtl::ClientSocket::Pointer &socket)
{
    igtl::StringMessage::Pointer transformStringMsg;
    transformStringMsg = igtl::StringMessage::New();
    std::stringstream transformStream;
    std::string transformString;
    transformStream << CMD_UID;
    // Set UID
    transformStringMsg->SetDeviceName(transformStream.str().c_str());
    transformStream.str(std::string());

    std::vector<double> transformVector = this->rosPoseToIGTL(T_FL_new);
    transformStream << "MoveToPose;" << "rob;";
    for(std::vector<double>::iterator it=transformVector.begin(); it!=transformVector.end();it++)
    {
        transformStream << *it << ";";
    }
    transformString = transformStream.str();
    // Set Transform as a String
    transformStringMsg->SetString(transformString);
    transformStream.str(std::string());
    transformStringMsg->Pack();
    CMD_UID++;
    return socket->Send(transformStringMsg->GetPackPointer(),transformStringMsg->GetPackSize());

}

std::vector<double> RosOpenIgtlBridge::rosPoseToIGTL(geometry_msgs::Pose pose)
{
    Eigen::Affine3d poseToEigen;
    tf::poseMsgToEigen(pose,poseToEigen);
    std::vector<double> returnValue;
    for(int row=0;row<3;row++)
    {
        for(int col; col<4; col++)
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
    return returnValue;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv, "RosOpenIGTLBridge");
    ros::NodeHandle RosOpenIGTLBridgeHandle(argv[1]);

    RosOpenIgtlBridge bridge(RosOpenIGTLBridgeHandle);
    return 0;
}
