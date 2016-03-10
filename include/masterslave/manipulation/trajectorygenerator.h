#ifndef TRAJECTORYGENERATOR_H
#define TRAJECTORYGENERATOR_H

#include "ros/ros.h"
#include "masterslave/Manipulation.h"
#include "masterslave/trajectorygeneratorConfig.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/Float64.h"

#include "Eigen/Core"

#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>

#include "masterslave/manipulation/trajectory/ptptraj.h"
#include "masterslave/manipulation/trajectory/circletraj.h"
#include "masterslave/manipulation/trajectory/trajectory.h"

#include <dynamic_reconfigure/server.h>
#include "masterslave/trajectorygeneratorConfig.h"

enum TRAJECTORY_STATE
{
    NO_STATE = -1,
    PTP,
    LINE,
    CIRCLE
};

class TrajectoryGenerator
{
public:
    TrajectoryGenerator(ros::NodeHandle& nh);
    ~TrajectoryGenerator();

private:
    bool trajectoryCallback(masterslave::Manipulation::Request& req, masterslave::Manipulation::Response& resp);
    void cycleTimeCallback(const std_msgs::Float64ConstPtr&);
    void configurationCallback(masterslave::trajectorygeneratorConfig &config, uint32_t level);
    ros::NodeHandle nh_;
    ros::ServiceServer trajectoryServer;
    ros::Subscriber cycleTimeSub;

    Eigen::Vector2i ptpTrajectory;
    int zCoordinate;
    int circleRadius;
    int trajectorySpeed;
    double cycleTime;

    bool start{false};
    bool startOld{false};

    std::unique_ptr<Trajectory> trajectoryGen;

    TRAJECTORY_STATE state{NO_STATE};
};

#endif // TRAJECTORYGENERATOR_H
