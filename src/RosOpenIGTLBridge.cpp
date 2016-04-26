#include "masterslave/RosOpenIGTLBridge.h"
#include "ros/ros.h"



#define M_TO_MM 1.0/1000.0f

#define MM_TO_M 1000


RosOpenIgtlBridge::RosOpenIgtlBridge(ros::NodeHandle nh): nh_(nh)
{

    dynamic_reconfigure::Server<masterslave::RosOpenIGTLBridgeConfig> server;
    dynamic_reconfigure::Server<masterslave::RosOpenIGTLBridgeConfig>::CallbackType f;
    f = boost::bind(&RosOpenIgtlBridge::configurationIGTLCallback,this ,_1,_2);
    server.setCallback(f);

    jointAngles = Eigen::VectorXd::Zero(7);
    jointAngles_new = Eigen::VectorXd::Zero(7);
    flangeTargetSub = nh_.subscribe("/flangeTarget",1,&RosOpenIgtlBridge::transformCallback,this);

    flangePub = nh_.advertise<geometry_msgs::PoseStamped>("/flangeLBR",1);

    for(int i=0; i < lbrJointAngleSub.size(); i++)
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



    ros::Timer timer = nh_.createTimer(ros::Duration(0.001), &RosOpenIgtlBridge::loop, this);
    ros::spin();
}

void RosOpenIgtlBridge::loop(const ros::TimerEvent& event)
{
    // Get pose of flange
    geometry_msgs::PoseStamped poseFLmsg;
    poseFLmsg.header.stamp = ros::Time::now();
    poseFLmsg.pose = poseFL;
    this->flangePub.publish(poseFLmsg);

    for(int i=0;i<7;i++)
    {
        sensor_msgs::JointState temp;
        temp.position.push_back(jointAngles[i]);
        lbrJointAnglePub[i].publish(temp);
    }


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
    rCommand=-10;
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
    }

    ROS_DEBUG_STREAM("rCommand: " << rCommand << " rTransform: " << rTransform);

    this->sendCommand(commandSocket_,"Idle;");
    ros::Rate rate(30);
    while(rCommand!=-1 && ros::ok())
    {
        if(sendTransformFlag)
        {
            sendTransformFlag = false;
            igtl::MessageHeader::Pointer messageHeaderCommand;
            messageHeaderCommand = igtl::MessageHeader::New();
            positionReached_ = this->positionReached(commandSocket_,messageHeaderCommand);
        }

        if(commandReceivedFromROS)
        {
            commandStringMutex_.lock();
            this->sendCommand(commandSocket_,openIGTLCommandString);
            commandStringMutex_.unlock();
            transformReceived_ = false;
        }
        rCommand = commandSocket_->GetConnected();
        rate.sleep();
    }
    commandSocket_->CloseSocket();
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
    igtl::Matrix4x4 T_FL;
    igtl::Matrix4x4 jointAnglesIGTL;
    igtl::TransformMessage::Pointer transformMsg;
    msgHeader->InitPack();
    stateUpdateMutex_.try_lock();
        rTransform = socket->Receive(msgHeader->GetPackPointer(),msgHeader->GetPackSize());
    stateUpdateMutex_.unlock();
    msgHeader->Unpack();
    transformMsg = igtl::TransformMessage::New();
    transformMsg->SetMessageHeader(msgHeader);
    transformMsg->AllocatePack();

    socket->Receive(transformMsg->GetPackBodyPointer(), transformMsg->GetPackBodySize());

    transformMsg->Unpack();
    if(strcmp(transformMsg->GetDeviceName(),"JointData")==0)
    {
        int cnt=0;


        //Empfang der Gelenkwinkel vom Visualisierungsinterface bzw. dem Socket
        transformMsg->GetMatrix(jointAnglesIGTL);
        jointAngleUpdateMutex_.lock();
        for(int i=0;i<4;i++)
        {
            for(int j=0;j<3;j++)
            {
                jointAngles[cnt] = jointAnglesIGTL[j][i];
                cnt++;
                if(cnt==7) break;
            }
            if(cnt==7) break;
        }
        jointAngleUpdateMutex_.unlock();
    }
    if(strcmp(transformMsg->GetDeviceName(),"T_EE")==0)
    {
        //ROS_DEBUG("T_EE empfangen");

        transformMsg->GetMatrix(T_FL);
        transformUpdateMutex_.lock();
        poseFL = this->igtlMatrixToRosPose(T_FL);
        transformUpdateMutex_.unlock();
        return 1;
    }
    return 0;
}


int RosOpenIgtlBridge::sendCommand(igtl::ClientSocket::Pointer &socket, std::string command)
{
    //if(!commandReceivedFromROS) return -1;
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
    ROS_INFO_STREAM("Send: \n" << command);

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

    sstream << poseToEigen.translation().x() << ";";
    sstream << poseToEigen.translation().y() << ";";
    sstream << poseToEigen.translation().z() << ";";
    for(int row=0;row<3;row++)
    {
        for(int col=0; col<3; col++)
        {
            sstream << poseToEigen.matrix()(row,col) << ";";
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
    stateUpdateMutex_.try_lock();
        if(rTransform==-1)
        {
            res.alive = false;
        }
        else
        {
            res.alive = true;
        }
    stateUpdateMutex_.unlock();
    //ROS_WARN_STREAM(req.state);
    stateString = req.state;
    sstream << stateString;

    // Checkt ob eine Transformation von ROS angekommen ist, die weitergeleitet werden kann
    //ROS_INFO_STREAM(stateString);
    //ROS_INFO_STREAM(rosTransformReceived_);
    if(rosTransformReceived_ && stateString == "MoveToPose;rob;")
    {
        transformUpdateMutex_.lock();
            sstream << rosPoseToIGTL(poseFL_new);
        transformUpdateMutex_.unlock();
        rosTransformReceived_ = false;
        commandReceivedFromROS = true;
    }
    // Checkt den Statestring und ob alle Gelenkwinkel empfangen wurden
    else if(stateString == "MoveToPose;rob;" && (jointAnglesCalled + 1 >> 7)==1)
    {
        jointAngleUpdateMutex_.lock();
        for(int i=0;i<jointAngles_new.rows();i++)
        {
            sstream << jointAngles_new(i) << ";";
        }
        jointAngleUpdateMutex_.unlock();
        jointAnglesCalled = 0;
        commandReceivedFromROS = true;
    }
    else if(stateString == "IDLE;" || stateString == "GravComp;")
    {
        commandReceivedFromROS = true;
    }
    commandStringMutex_.lock();
        openIGTLCommandString = sstream.str();
   commandStringMutex_.unlock();
    //ROS_WARN_STREAM("Commandstring: " << openIGTLCommandString);
    transformReceived_ = true;
    sstream.str(std::string());
    stateServiceCalled_ = true;
    return true;
}

void RosOpenIgtlBridge::lbrJointAngleCallback(const std_msgs::Float64ConstPtr &jointAngle, int number)
{
    jointAngles_new(number) = jointAngle->data;

    // Es wird geguckt, ob alle Gelenke schon empfangen wurden
    if(jointAnglesCalled+1 < 1 << 7)
    {
        // Binäre Operation (Bits der Gelenknummern zu 1 gesetzt)
        jointAnglesCalled += 1 << number;
    }
}

void RosOpenIgtlBridge::configurationIGTLCallback(masterslave::RosOpenIGTLBridgeConfig &config, uint32_t level)
{
    if(config.StartOpenIGTL)
    {
        //Insgesamt 3 Threads um ROS von OpenIGTL zu entkoppeln
        boost::thread(boost::bind(&RosOpenIgtlBridge::openIGTLinkTransformThread,this));
        boost::thread(boost::bind(&RosOpenIgtlBridge::openIGTLinkThread,this));
    }
}

int main(int argc, char** argv)
{
    ros::init(argc,argv, "RosOpenIGTLBridge");
    ros::NodeHandle RosOpenIGTLBridgeHandle;

    RosOpenIgtlBridge bridge(RosOpenIGTLBridgeHandle);
    return 0;
}
