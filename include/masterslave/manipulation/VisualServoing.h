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

class VisualServoing {
public:
    VisualServoing(ros::NodeHandle &nh);
private:
    void markerCallback(const ar_track_alvar_msgs::AlvarMarkersConstPtr& markerPtr);
    bool visualServoingCallback(masterslave::Manipulation::Request& req, masterslave::Manipulation::Response& resp);
    ros::NodeHandle nh_;
    ros::Subscriber markerSub;
    ros::ServiceServer visualServoingServer;
    Eigen::Affine3d initialTransform;
    Eigen::Affine3d actualTransform;
    Eigen::Affine3d differenceTransform;
    std::array<int,2> ids = {{0,1}};
    bool initialRun = true;
};

#endif // VISUALSERVOING_H
