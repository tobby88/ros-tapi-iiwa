#ifndef URSULAKINEMATICS_H
#define URSULAKINEMATICS_H

#include "kinematics.h"
#include "eiquadprog.hpp"
#include <eigen3/Eigen/Core>
#include "ros/ros.h"
#include "masterslave/rcmTest.h"
#include "masterslave/directKinematics.h"
#include "masterslave/inverseKinematics.h"

#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float64.h>

class UrsulaKinematics: public Kinematics
{
public:
    UrsulaKinematics(ros::NodeHandle&);
private:

    ros::NodeHandle nh_;

    Eigen::VectorXd calcDirKin(Eigen::VectorXd);

    Eigen::MatrixXd calcAnalyticalJacobian(Eigen::VectorXd jointAngles);
    void calcInvKin(Eigen::Affine3d);
    //Optimization constraint methods
    Eigen::VectorXd angleMonitoring(Eigen::VectorXd deltaQ, Eigen::VectorXd q, double Hmax, double& a);
    Eigen::MatrixXd collisionControl(Eigen::VectorXd q);
    Eigen::VectorXd trocarMonitoring(Eigen::VectorXd qAct, Eigen::VectorXd deltaQ, Eigen::MatrixXd& A); // and RCM is needed
    Eigen::MatrixXd minimizeVelocities(double, Eigen::MatrixXd);
    Eigen::MatrixXd minimizeAcceleration(double cycleTime, Eigen::MatrixXd weightMatrix, Eigen::VectorXd deltaQ, Eigen::MatrixXd& a);

    Eigen::Affine3d buildAffine3d(const Eigen::Vector3d &translXYZ, const Eigen::Vector3d &axisZYX, bool zyx);


    ros::ServiceServer rcmServiceServer;
    ros::ServiceServer directKinematicsServer;
    ros::ServiceServer inverseKinematicsServer;
    ros::Subscriber cycleTimeSub;

    bool rcmCallback(masterslave::rcmTest::Request&, masterslave::rcmTest::Response&);
    bool directKinematicsCallback(masterslave::directKinematics::Request&, masterslave::directKinematics::Response&);
    bool inverseKinematicsCallback(masterslave::inverseKinematics::Request&, masterslave::inverseKinematics::Response&);
    void cycleTimeCallback(const std_msgs::Float64ConstPtr&);

    //geometric description parameters of the LBR iiwa 14 R820
    static const lbrDescriptionParameters LBR_PARAMETERS;

    static const toolDescriptionParameters TOOL_PARAMETERS;

    //geometric description parameters of the LBR iiwa 14 R820 (angle limits and speed limits)
    Eigen::Matrix<double, 10, 1> URSULA_MAX_ANGLES;

    Eigen::Matrix<double, 10, 1> URSULA_MAX_ANGLES_SPEED;

    Eigen::Matrix<double, 10, 10> jointWeightMatrix;

    // Parameters for constraints
    static const double minDistance = 0.05; // minimal distance between some objects before collision control starts working

    //max iterations for inverse kinematics
    static const int maxIterations = 10;

    //last cycle time
    double cycleTime;

    static const double residualMax= 10000;

    //Endeffector Position in translation and rotation in euler angles (DLR-Convention)
    Eigen::Matrix<double, 6, 1> curEEPosition;
    Eigen::Matrix<double, 6, 1> desEEPosition;

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

    Eigen::Affine3d RCM;

    Eigen::MatrixXd geomJacobian;
    Eigen::MatrixXd analyticalJacobian;

    Eigen::VectorXd rotation2RPY(Eigen::Affine3d);
};

#endif // URSULAKINEMATICS_H
