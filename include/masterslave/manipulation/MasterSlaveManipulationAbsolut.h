#ifndef MASTERSLAVEMANIPULATIONABSOLUT_H
#define MASTERSLAVEMANIPULATIONABSOLUT_H

#include <algorithm>
#include <array>

#include "ros/ros.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "masterslave/Manipulation.h"

#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float64.h"
#include "Eigen/Dense"

#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>

#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>

/**
 * @file MasterSlaveManipulationAbsolute.h
 *
 * @class MasterSlaveManipulationAbsolute
 * @brief Klasse, die sich um die Manipulation des TCP mittels des Markertrackings kümmert
 *
 * @author Fabian Baier
 * @date 04.04.2016
 */

class MasterSlaveManipulationAbsolute
{
public:
    MasterSlaveManipulationAbsolute(ros::NodeHandle &nh);
private:
    void markerCallback(const ar_track_alvar_msgs::AlvarMarkersConstPtr &handMarker, const ar_track_alvar_msgs::AlvarMarkersConstPtr &referenceMarker);
    void cycleTimeCallback(const std_msgs::Float64ConstPtr& val);
    bool masterSlaveCallback(masterslave::Manipulation::Request& req, masterslave::Manipulation::Response& resp);
    ros::NodeHandle nh_;
    message_filters::Subscriber<ar_track_alvar_msgs::AlvarMarkers> markerSub;
    message_filters::Subscriber<ar_track_alvar_msgs::AlvarMarkers> markerSubRef;
    ros::Subscriber cycleTimeSub;
    ros::ServiceServer masterSlaveServer;
    double cycleTime;
    Eigen::Affine3d poseAct;
    Eigen::Affine3d poseOld;
    Eigen::Affine3d difference;
    bool initialRun{true};

    double frameTime;
    double lastTime;
};

#endif // MASTERSLAVEMANIPULATIONABSOLUT_H
