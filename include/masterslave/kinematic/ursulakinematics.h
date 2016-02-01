#ifndef URSULAKINEMATICS_H
#define URSULAKINEMATICS_H

#include "kinematics.h"

class UrsulaKinematics: public Kinematics
{
public:
    UrsulaKinematics(const Eigen::Affine3d);
private:
    void calcDirKin();
    void calcAnalyticalJacobian();
    void calcInvKin();
    //geometric description parameters of the LBR iiwa 14 R820
    static const lbrDescriptionParameters LBR_PARAMETERS;
    //geometric description parameters of the LBR iiwa 14 R820 (angle limits and speed limits)
    static const lbrJointAngles LBR_MAX_ANGLES;
    static const lbrJointAngles LBR_MIN_ANGLES;
    static const lbrJointAngles LBR_MAX_ANGLES_SPEED;
    static const lbrJointAngles LBR_MIN_ANGLES_SPEED;


    //joint angles of the LBR iiwa 14 R820
    lbrJointAngles lbrJointAnglesAct;

    Eigen::Affine3d T_0_Q1;
    Eigen::Affine3d T_0_Q2;
    Eigen::Affine3d T_0_Q3;
    Eigen::Affine3d T_0_Q4;
    Eigen::Affine3d T_0_Q5;
    Eigen::Affine3d T_0_Q6;
    Eigen::Affine3d T_0_Q7;
    Eigen::Affine3d T_0_FL;

    Eigen::MatrixXd geomJacobian;
    Eigen::MatrixXd analyticalJacobian;


    bool isDirKinCalced;
};

#endif // URSULAKINEMATICS_H
