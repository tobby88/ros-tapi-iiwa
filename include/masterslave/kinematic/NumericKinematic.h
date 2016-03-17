#ifndef NUMERICKINEMATIC_H
#define NUMERICKINEMATIC_H
#include "IKinematic.h"

#include <eigen3/Eigen/Core>
#include "ros/ros.h"


//service definitions
#include "masterslave/NumericKinematicRCM.h"
#include "masterslave/NumericKinematicDirectKinematics.h"
#include "masterslave/NumericKinematicInverseKinematics.h"

#include "masterslave/NumericKinematicConfig.h"


//dynamic reconfigure configuration file



#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float64.h>

//solver algorithm for quadratic programming problems (very nice perfomance)
#include "eiquadprog.hpp"

class NumericKinematic: public IKinematic
{
public:
    NumericKinematic(ros::NodeHandle&);
private:

    ros::NodeHandle nh_;

    Eigen::VectorXd calcDirKin(Eigen::VectorXd);

    Eigen::MatrixXd calcAnalyticalJacobian(Eigen::VectorXd jointAngles);

    bool calcInvKin(Eigen::Affine3d);

    //Optimization constraint methods
    Eigen::VectorXd angleMonitoring(Eigen::VectorXd deltaQ, Eigen::VectorXd q, double Hmax, double& a);
    Eigen::MatrixXd collisionControl(Eigen::VectorXd q);
    Eigen::VectorXd trocarMonitoring(Eigen::VectorXd qAct, Eigen::VectorXd deltaQ, Eigen::MatrixXd& A); // and RCM is needed
    Eigen::MatrixXd minimizeVelocities(double, Eigen::MatrixXd);
    Eigen::MatrixXd minimizeAcceleration(double cycleTime, Eigen::MatrixXd weightMatrix, Eigen::VectorXd deltaQ, Eigen::MatrixXd& a);
    Eigen::VectorXd avoidSingularities(Eigen::VectorXd qAct, Eigen::VectorXd qOffset, double weight, double& a);

    //Subscriber and Sevice Servers
    ros::ServiceServer rcmServiceServer;
    ros::ServiceServer directKinematicsServer;
    ros::ServiceServer inverseKinematicsServer;
    ros::Subscriber cycleTimeSub;

    //Callbacks
    bool rcmCallback(masterslave::NumericKinematicRCM::Request&, masterslave::NumericKinematicRCM::Response&);
    bool directKinematicsCallback(masterslave::NumericKinematicDirectKinematics::Request&, masterslave::NumericKinematicDirectKinematics::Response&);
    bool inverseKinematicsCallback(masterslave::NumericKinematicInverseKinematics::Request&, masterslave::NumericKinematicInverseKinematics::Response&);
    void cycleTimeCallback(const std_msgs::Float64ConstPtr&);
    void configurationCallback(masterslave::NumericKinematicConfig& config, uint32_t level);

    //geometric description parameters of the LBR iiwa 14 R820
    const lbrDescriptionParameters LBR_PARAMETERS = { 0.160, 0.200, 0.200, 0.220, 0.180, 0.220, 0.076, 0.050};
    //geometric description parameters of the TOOL
    const toolDescriptionParameters TOOL_PARAMETERS = {0.438, 0.0, 0.062, 0.0, 90.0, 0.0, 0.0088, 0.017, 0.305};

    //geometric description parameters of the LBR iiwa 14 R820 (angle limits and speed limits)
    Eigen::Matrix<double, 10, 1> URSULA_MAX_ANGLES;
    Eigen::Matrix<double, 10, 1> URSULA_MAX_ANGLES_SPEED;

    //in according to LBR Specifications (in Degree)
    const double MAX_ANGLES[10] = {170*DEG_TO_RAD, 120*DEG_TO_RAD, 170*DEG_TO_RAD, 120*DEG_TO_RAD, 170*DEG_TO_RAD, 120*DEG_TO_RAD, 175*DEG_TO_RAD, 85*DEG_TO_RAD, 90*DEG_TO_RAD, 90*DEG_TO_RAD};
    const double MAX_ANGLES_SPEED[10] = { 85*DEG_TO_RAD, 85*DEG_TO_RAD, 100*DEG_TO_RAD, 75*DEG_TO_RAD, 130*DEG_TO_RAD, 135*DEG_TO_RAD, 135*DEG_TO_RAD, 135*DEG_TO_RAD, 135*DEG_TO_RAD, 135*DEG_TO_RAD};

    //weightMatrix for the usage of different joints
    Eigen::Matrix<double, 10, 10> jointWeightMatrix;

    // Parameters for constraints
    const double minDistance{0.05}; // minimal distance between some objects before collision control starts working

    //max iterations for inverse kinematics
    const int maxIterations{10};

    //last cycle time
    double cycleTime;

    //Endeffector Position in translation and rotation in euler angles (DLR-Convention)
    Eigen::Matrix<double, 6, 1> curEEPosition;
    Eigen::Matrix<double, 6, 1> desEEPosition;

    // Direct Kinematics (Array instead?!)
    Eigen::Affine3d T_0_Q1;
    Eigen::Affine3d T_0_Q2;
    Eigen::Affine3d T_0_Q3;
    Eigen::Affine3d T_0_Q4;
    Eigen::Affine3d T_0_Q5;
    Eigen::Affine3d T_0_Q6;
    Eigen::Affine3d T_0_Q7;
    Eigen::Affine3d T_0_FL;
    Eigen::Affine3d T_0_SCH;
    Eigen::Affine3d T_0_Q8;
    Eigen::Affine3d T_0_Q9;
    Eigen::Affine3d T_0_Q10;

    //weight parameters for online configuration
    double collisionAvoidanceGain{0.0000001};
    double angleMonitoringGain{0.0000001};
    double singularityGain{0.000001};
    double accelerationGain{0.0000001};
    double velocityGain{0.0000001};
    double maxSpeed{0.7};
    double rescueFactor{0};
    double trocarGain{1};
    double tcpGain{1};

    // Jacobians
    Eigen::MatrixXd geomJacobian;
    Eigen::MatrixXd analyticalJacobian;

    // Helper Method
    Eigen::VectorXd rotation2RPY(Eigen::Affine3d);
    Eigen::Affine3d buildAffine3d(const Eigen::Vector3d &translXYZ, const Eigen::Vector3d &axisZYX, bool zyx);
};

#endif // NumericKinematic_H
