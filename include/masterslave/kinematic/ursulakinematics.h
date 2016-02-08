#ifndef URSULAKINEMATICS_H
#define URSULAKINEMATICS_H

#include "kinematics.h"
#include "eiquadprog.hpp"
#include <eigen3/Eigen/Core>

class UrsulaKinematics: public Kinematics
{
public:
    UrsulaKinematics(Eigen::Affine3d);
    Eigen::Affine3d getT_0_Q8(){ return T_0_Q8;}
    Eigen::Affine3d getFlange(){ return T_0_FL;}
    void setT_0_EE(Eigen::Affine3d);
    void setAngles(Eigen::VectorXd);
    void setToolAngles(Eigen::VectorXd);
    Eigen::Affine3d calcStartPos(Eigen::Affine3d, Eigen::VectorXd);

private:

    Eigen::VectorXd calcDirKin(Eigen::VectorXd);

    Eigen::MatrixXd calcAnalyticalJacobian(Eigen::VectorXd jointAngles);
    void calcInvKin();
    Eigen::MatrixXd angleMonitoring(Eigen::VectorXd q, double Hmax);
    Eigen::MatrixXd collisionControl(Eigen::VectorXd q);
    Eigen::MatrixXd trocarMonitoring(Eigen::VectorXd q); // and RCM is needed

    //geometric description parameters of the LBR iiwa 14 R820
    static const lbrDescriptionParameters LBR_PARAMETERS;

    //geometric description parameters of the LBR iiwa 14 R820 (angle limits and speed limits)
    Eigen::Matrix<double, 10, 1> URSULA_MAX_ANGLES;

    Eigen::Matrix<double, 10, 1> URSULA_MAX_ANGLES_SPEED;


    // Parameters for constraints
    static const double minDistance = 0.05; // minimal distance between some objects before collision control starts working

    //max iterations for inverse kinematics
    static const int maxIterations = 50;

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
    Eigen::Affine3d T_0_Q8;
    Eigen::Affine3d T_0_Q9;
    Eigen::Affine3d T_0_Q10;

    Eigen::MatrixXd geomJacobian;
    Eigen::MatrixXd analyticalJacobian;


    bool isDirKinCalced;

    Eigen::VectorXd rotation2RPY(Eigen::Affine3d);
};

#endif // URSULAKINEMATICS_H
