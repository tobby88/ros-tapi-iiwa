#ifndef ROSOPENIGTLBRIDGE_H
#define ROSOPENIGTLBRIDGE_H

#include <ros/ros.h>
#include <stdlib.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include <dynamic_reconfigure/server.h>
#include <masterslave/rosigtlbridgeConfig.h>
#include "masterslave/state.h"

// for own thread
#include "ros/callback_queue.h"
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include "boost/thread/mutex.hpp"

// for own OpenIGTLink
#include "igtlOSUtil.h"
#include "igtlMessageHeader.h"
#include "igtlTransformMessage.h"
#include "igtlClientSocket.h"
#include "igtlStatusMessage.h"
#include "igtlStringMessage.h"
#include "igtl_transform.h"

#include <sstream>
#include "Eigen/Eigen"
#include "Eigen/Dense"
#include <eigen_conversions/eigen_msg.h>

class RosOpenIgtlBridge
{
public:
    RosOpenIgtlBridge(ros::NodeHandle);
    bool stateService(masterslave::state::Request&, masterslave::state::Response&);
private:
    void transformCallback(geometry_msgs::PoseStampedConstPtr);
    void openIGTLinkThread();
    void openIGTLinkTransformThread();
    // command socket methods
    int sendCommand(igtl::ClientSocket::Pointer&, std::string);
    int positionReached(igtl::ClientSocket::Pointer&, igtl::MessageBase::Pointer&);

    // transform socket methods
    int receiveTransform(igtl::ClientSocket::Pointer&, igtl::MessageBase::Pointer&);

    //transmission method
    std::string rosPoseToIGTL(geometry_msgs::Pose);
    geometry_msgs::Pose igtlMatrixToRosPose(igtl::Matrix4x4&);

    //dynamic reconfigure
    void configurationIGTLCallback(masterslave::rosigtlbridgeConfig &config, uint32_t level);



    ros::Subscriber flangeTargetSub;
    ros::Publisher flangePub;
    ros::ServiceServer stateServiceServer;
    ros::NodeHandle nh_;
    boost::mutex update_mutex_;
    boost::thread openIGTLThread;

    int positionReached_;
    bool sendTransformFlag;

    boost::mutex transformUpdateMutex_;

    igtl::Matrix4x4 T_FL;
    igtl::Matrix4x4 T_FL_new;
    geometry_msgs::Pose poseFL;
    geometry_msgs::Pose poseFL_new;

    igtl::ClientSocket::Pointer commandSocket_;
    static const int COMMAND_PORT = 49001;
    static const char* COMMAND_IP;
    int rCommand;

    igtl::ClientSocket::Pointer transformSocket_;
    static const int TRANSFORM_PORT = 49002;
    static const char* TRANSFORM_IP;
    int rTransform;

    unsigned long long CMD_UID;

    std::string openIGTLCommandString;
    std::string stateString;

    double sampleTime_;

    bool stop_;
    bool start_;
    bool transformReceived_;
    bool rosTransformReceived_;

    static const unsigned int CONNECTION_TIMEOUT=30;

};

#endif // ROSOPENIGTLBRIDGE_H
