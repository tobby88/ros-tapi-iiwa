#ifndef ROSOPENIGTLBRIDGE_H
#define ROSOPENIGTLBRIDGE_H

#include <ros/ros.h>
#include <stdlib.h>
#include "geometry_msgs/PoseStamped.h"
#include <dynamic_reconfigure/server.h>

// for own thread
#include "ros/callback_queue.h"
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
private:
    void transformCallback(geometry_msgs::PoseStampedConstPtr);
    void openIGTLinkThread();
    // command socket methods
    int sendTransform(igtl::ClientSocket::Pointer&);
    int sendIdleState(igtl::ClientSocket::Pointer&);
    int positionReached(igtl::ClientSocket::Pointer&, igtl::MessageBase::Pointer&);

    // transform socket methods
    int receiveTransform(igtl::ClientSocket::Pointer&, igtl::MessageBase::Pointer&);

    //transmission method
    std::vector<double> rosPoseToIGTL(geometry_msgs::Pose);
    geometry_msgs::Pose igtlMatrixToRosPose(igtl::Matrix4x4&);

    ros::Subscriber flangeTargetSub;
    ros::Publisher flangePub;
    ros::NodeHandle nh_;
    boost::mutex update_mutex_;
    boost::thread openIGTLThread;

    int positionReached_;
    bool sendTransformFlag;

    igtl::ClientSocket::Pointer transformSocket_;
    int transformPort_;
    std::string transformIP_;
    int rTransform;
    igtl::Matrix4x4 T_FL;
    geometry_msgs::Pose poseFL;
    geometry_msgs::Pose T_FL_new;

    igtl::ClientSocket::Pointer commandSocket_;
    int commandPort_;
    std::string commandIP_;
    int rCommand;
    int CMD_UID;

    std::string openIGTLCommandString;

    double sampleTime_;


};

#endif // ROSOPENIGTLBRIDGE_H
