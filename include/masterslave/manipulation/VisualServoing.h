#ifndef VISUALSERVOING_H
#define VISUALSERVOING_H

#include "ros/ros.h"
#include "Eigen/Dense"
#include "masterslave/Manipulation.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"

#include <array>

#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>

#include <dynamic_reconfigure/server.h>
#include "masterslave/VisualServoingConfig.h"

class VisualServoing {
public:
    VisualServoing(ros::NodeHandle &nh);
private:
    void markerCallback(const ar_track_alvar_msgs::AlvarMarkersConstPtr& markerPtr);
    void configurationCallback(masterslave::VisualServoingConfig& config, uint32_t level);
    bool visualServoingCallback(masterslave::Manipulation::Request& req, masterslave::Manipulation::Response& resp);
    Eigen::Vector3d calculateTranslationalPID();
    Eigen::Matrix3d calculateRotationalPID();
    ros::NodeHandle nh_;
    ros::Subscriber markerSub;
    ros::ServiceServer visualServoingServer;
    Eigen::Affine3d initialTransform;
    Eigen::Affine3d actualTransform;
    Eigen::Affine3d differenceTransform;
    Eigen::Affine3d differenceTransformOld;
    std::array<int,2> ids = {{0,1}};
    bool initialRun = true;

    double cycleTime{0.033};

    double pTrans{1};
    double dTrans{0.001};
    double iTrans{0.0001};

    Eigen::Vector3d integralErrorTrans{0,0,0};

    double pRot{1};
    double dRot{0};
    double iRot{0};

    Eigen::Vector3d integralErrorRot;

};

#endif // VISUALSERVOING_H
