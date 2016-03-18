#ifndef TRAJECTORYGENERATOR_H
#define TRAJECTORYGENERATOR_H

#include "ros/ros.h"
#include "masterslave/Manipulation.h"


#include "geometry_msgs/Pose.h"
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/Float64.h"

#include "Eigen/Core"

#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>

#include "masterslave/manipulation/trajectory/LineTrajectory.h"
#include "masterslave/manipulation/trajectory/PTPTrajectory.h"
#include "masterslave/manipulation/trajectory/CircleTrajectory.h"
#include "masterslave/manipulation/trajectory/ITrajectory.h"

#include <dynamic_reconfigure/server.h>
#include "masterslave/TrajectoryGeneratorConfig.h"

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
    void configurationCallback(masterslave::TrajectoryGeneratorConfig &config, uint32_t level);
    ros::NodeHandle nh_;
    ros::ServiceServer trajectoryServer;
    ros::Subscriber cycleTimeSub;

    Eigen::Vector3d rcm;
    Eigen::Affine3d startPosition;
    Eigen::Vector3d firstPoint;
    Eigen::Vector3d secondPoint;
    double zCoordinate;
    double circleRadius;
    double trajectorySpeed;
    double cycleTime;

    bool start{false};
    bool rcmPositionThere{false};

    const double MM_TO_M{0.001};

    std::unique_ptr<ITrajectory> trajectoryGen;

    TRAJECTORY_STATE state{NO_STATE};
};

#endif // TRAJECTORYGENERATOR_H
