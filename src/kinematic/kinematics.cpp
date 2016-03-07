#include "masterslave/kinematic/kinematics.h"

#define DEG_TO_RAD M_PI/180

void Kinematics::setT_0_EE(Eigen::Affine3d value)
{
    T_0_EE = value;
}

void Kinematics::setRCM(const Eigen::Affine3d startPose)
{
    Eigen::Affine3d T_FL_RCM = buildAffine3d(Eigen::Vector3d(TOOL_PARAMETERS.X_RCM,TOOL_PARAMETERS.Y_0_Q4,TOOL_PARAMETERS.Z_0_Q4),Eigen::Vector3d(TOOL_PARAMETERS.A_0_Q4*DEG_TO_RAD,TOOL_PARAMETERS.B_0_Q4*DEG_TO_RAD,TOOL_PARAMETERS.C_0_Q4*DEG_TO_RAD),true);
    RCM = startPose*T_FL_RCM;
}


