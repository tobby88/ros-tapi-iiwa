#include "masterslave/kinematic/laparoscope.h"

#define DEG_TO_RAD M_PI/180



Laparoscope::Laparoscope(Eigen::Affine3d startPositionLBR)
{

    setRCM(startPositionLBR);
    ROS_WARN_STREAM_NAMED("Remote Center of Motion","Position: " << RCM.translation());

}

void Laparoscope::setAngles(const toolAngles value)
{
    toolAnglesAct = value;
    calcDirKin();
}

void Laparoscope::calcDirKin()
{
    T_FL_EE = calcLaparoscopeDirKin();
}


void Laparoscope::calcInvKin()
{
    Eigen::Affine3d T_0_Q6 = T_0_EE.translate(Eigen::Vector3d(-TOOL_PARAMETERS.L_Q6_EE,0,0));
    Eigen::Vector3d p_RCM_Q6 = T_0_Q6.translation()-RCM.translation();
    Eigen::Vector4d z_Q6_h = T_0_Q6.matrix()*Eigen::Vector4d(0,0,1,0);
    Eigen::Vector3d z_Q6;
    z_Q6 << z_Q6_h(0), z_Q6_h(1), z_Q6_h(2);
    Eigen::Vector3d nPlane = p_RCM_Q6.cross(z_Q6);

    Eigen::Vector4d y_Q6_h = T_0_Q6.matrix()*Eigen::Vector4d(0,-1,0,0);
    Eigen::Vector3d y_Q6;
    y_Q6 << y_Q6_h(0), y_Q6_h(1), y_Q6_h(2);

    toolAnglesTar.Q6 = acos(nPlane.dot(y_Q6)/(nPlane.norm()*y_Q6.norm()));

    if(isnan(toolAnglesTar.Q6))
    {
        toolAnglesTar.Q6 = 0.0;
    }

    if(toolAnglesTar.Q6>M_PI/2)
    {
        toolAnglesTar.Q6 = M_PI/2;
    }

    if(nPlane.cross(y_Q6).dot(z_Q6)<0)
    {
        toolAnglesTar.Q6 = -toolAnglesTar.Q6;
    }

    Eigen::Affine3d T_EE_Q5 = Kinematics::buildAffine3d(Eigen::Vector3d::Zero(),Eigen::Vector3d(90*DEG_TO_RAD,0,-toolAnglesTar.Q6),true);
    T_EE_Q5.translate(Eigen::Vector3d(-TOOL_PARAMETERS.L_Q5_Q6,0,0));
    Eigen::Affine3d T_0_Q5 = T_0_EE*T_EE_Q5;

    Eigen::Vector4d RCM_x_h = RCM.matrix()*Eigen::Vector4d(0,0,1,0);
    Eigen::Vector3d RCM_x;
    RCM_x << RCM_x_h(0), RCM_x_h(1), RCM_x_h(2);

    Eigen::Vector4d z_Q5_h = T_0_Q5.matrix() * Eigen::Vector4d(0,0,1,0);
    Eigen::Vector3d z_Q5;
    z_Q5 << z_Q5_h(0), z_Q5_h(1), z_Q5_h(2);

    Eigen::Vector4d y_Q5_h = T_0_Q5.matrix() * Eigen::Vector4d(0,-1,0,0);
    Eigen::Vector3d y_Q5;
    y_Q5 << y_Q5_h(0), y_Q5_h(1), y_Q5_h(2);


    Eigen::Vector3d p_Q5_RCM = T_0_Q5.translation() - RCM.translation();

    Eigen::Vector3d x_Q4 = p_Q5_RCM.cross(z_Q5);

    toolAnglesTar.Q5 = acos(x_Q4.dot(y_Q5)/(x_Q4.norm()*y_Q5.norm()));

    if(isnan(toolAnglesTar.Q5))
    {
        toolAnglesTar.Q5 = 0.0;
    }

    if(toolAnglesTar.Q5>M_PI/2)
    {
        toolAnglesTar.Q5 = M_PI/2;
    }

    if(x_Q4.cross(y_Q5).dot(z_Q5)<0)
    {
        toolAnglesTar.Q5 = -toolAnglesTar.Q5;
    }

    Eigen::Affine3d T_Q5_Q4 = Kinematics::buildAffine3d(Eigen::Vector3d::Zero(),Eigen::Vector3d(-90*DEG_TO_RAD,0,-90*DEG_TO_RAD-toolAnglesTar.Q5),true);
    T_0_Q4 = T_0_Q5*T_Q5_Q4;

    Eigen::Vector3d p_Q4_RCM = T_0_Q4.translation() - RCM.translation();

    Eigen::Vector4d y_FL_h = T_0_FL.matrix() * Eigen::Vector4d(0,1,0,0);
    Eigen::Vector3d y_FL;
    y_FL << y_FL_h(0), y_FL_h(1), y_FL_h(2);

    Eigen::Vector3d z_FL = y_FL.cross(p_Q4_RCM);

    toolAnglesTar.Q4 = acos(z_FL.dot(x_Q4)/(z_FL.norm()*x_Q4.norm()));

    if(isnan(toolAnglesTar.Q4))
    {
        toolAnglesTar.Q4  = 0.0;
    }

    if(z_FL.cross(x_Q4).dot(p_Q4_RCM)<0)
    {
        toolAnglesTar.Q4 = -toolAnglesTar.Q4;
    }

    Eigen::Affine3d T_Q4_FL = Kinematics::buildAffine3d(Eigen::Vector3d::Zero(),Eigen::Vector3d(-TOOL_PARAMETERS.A_0_Q4*DEG_TO_RAD,-TOOL_PARAMETERS.B_0_Q4*DEG_TO_RAD,-TOOL_PARAMETERS.C_0_Q4*DEG_TO_RAD-toolAnglesTar.Q4),true);
    T_Q4_FL.translate(Eigen::Vector3d(-TOOL_PARAMETERS.X_0_Q4,-TOOL_PARAMETERS.Y_0_Q4,-TOOL_PARAMETERS.Z_0_Q4));
    T_0_FL = T_0_Q4*T_Q4_FL;
}
