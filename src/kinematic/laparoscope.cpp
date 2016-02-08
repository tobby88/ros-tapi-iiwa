#include "masterslave/kinematic/laparoscope.h"

void Laparoscope::setAngles(const Eigen::VectorXd value)
{
    jointAnglesAct = value;
    calcDirKin();
}


Laparoscope::Laparoscope(Eigen::Affine3d startPositionLBR)
{

    setRCM(startPositionLBR);
    ROS_WARN_STREAM_NAMED("Remote Center of Motion","Position: " << RCM.translation());
    jointAnglesAct = Eigen::VectorXd(3);
    jointAnglesTar = Eigen::VectorXd(3);
}

void Laparoscope::calcDirKin()
{
    T_FL_Q4 = buildAffine3d(Eigen::Vector3d(TOOL_PARAMETERS.X_0_Q4,TOOL_PARAMETERS.Y_0_Q4,TOOL_PARAMETERS.Z_0_Q4),Eigen::Vector3d(TOOL_PARAMETERS.A_0_Q4*DEG_TO_RAD,TOOL_PARAMETERS.B_0_Q4*DEG_TO_RAD,TOOL_PARAMETERS.C_0_Q4*DEG_TO_RAD),true);
    T_FL_Q4.rotate(Eigen::AngleAxis<double>(jointAnglesAct(0),Eigen::Vector3d::UnitZ()));
    Eigen::Affine3d T_Q4_Q5 = buildAffine3d(Eigen::Vector3d::Zero(),Eigen::Vector3d(90*DEG_TO_RAD,0,90*DEG_TO_RAD+jointAnglesAct(1)),false);
    /* Fehler: Zuerst um Q5 drehen, dann verschieben */
    Eigen::Affine3d T_Q5_Q6 = buildAffine3d(Eigen::Vector3d(TOOL_PARAMETERS.L_Q5_Q6,0,0),Eigen::Vector3d(-90*DEG_TO_RAD,0,0),true);
    Eigen::Affine3d T_Q6_EE = buildAffine3d(Eigen::Vector3d::Zero(),Eigen::Vector3d(0,0,jointAnglesAct(2)),true);
    T_Q6_EE.translate(Eigen::Vector3d(TOOL_PARAMETERS.L_Q6_EE,0,0));

    T_FL_Q5 = T_FL_Q4*T_Q4_Q5;
    T_FL_Q6 = T_FL_Q5*T_Q5_Q6;

    T_FL_EE = T_FL_Q4*T_Q4_Q5*T_Q5_Q6*T_Q6_EE;
}


void Laparoscope::calcInvKin()
{
    Eigen::Affine3d T_0_Q6 = T_0_EE.translate(Eigen::Vector3d(-TOOL_PARAMETERS.L_Q6_EE,0,0));
    Eigen::Vector3d p_RCM_Q6 = T_0_Q6.translation()-RCM.translation();
    Eigen::Vector3d z_Q6 = T_0_Q6.matrix().col(3).head(3);
    Eigen::Vector3d nPlane = p_RCM_Q6.cross(z_Q6);

    Eigen::Vector3d y_Q6 = -T_0_Q6.matrix().col(2).head(3);

    jointAnglesTar(2) = acos(nPlane.dot(y_Q6)/(nPlane.norm()*y_Q6.norm()));

    if(isnan(jointAnglesTar(2)))
    {
        jointAnglesTar(2) = 0.0;
    }

    if(jointAnglesTar(2)>M_PI/2)
    {
        jointAnglesTar(2) = M_PI/2;
    }

    if(nPlane.cross(y_Q6).dot(z_Q6)<0)
    {
        jointAnglesTar(2) = -jointAnglesTar(2);
    }

    Eigen::Affine3d T_EE_Q5 = Kinematics::buildAffine3d(Eigen::Vector3d::Zero(),Eigen::Vector3d(90*DEG_TO_RAD,0,-jointAnglesTar(2)),true);
    T_EE_Q5.translate(Eigen::Vector3d(-TOOL_PARAMETERS.L_Q5_Q6,0,0));
    Eigen::Affine3d T_0_Q5 = T_0_EE*T_EE_Q5;

    Eigen::Vector3d RCM_x = RCM.matrix().col(3).head(3);

    Eigen::Vector3d z_Q5 = T_0_Q5.matrix().col(3).head(3);

    Eigen::Vector3d y_Q5 = -T_0_Q5.matrix().col(3).head(3);

    Eigen::Vector3d p_Q5_RCM = T_0_Q5.translation() - RCM.translation();

    Eigen::Vector3d x_Q4 = p_Q5_RCM.cross(z_Q5);

    jointAnglesAct(1) = acos(x_Q4.dot(y_Q5)/(x_Q4.norm()*y_Q5.norm()));

    if(isnan(jointAnglesTar(1)))
    {
        jointAnglesTar(1) = 0.0;
    }

    if(jointAnglesTar(1)>M_PI/2)
    {
        jointAnglesTar(1) = M_PI/2;
    }

    if(x_Q4.cross(y_Q5).dot(z_Q5)<0)
    {
        jointAnglesTar(1) = -jointAnglesTar(1);
    }

    Eigen::Affine3d T_Q5_Q4 = Kinematics::buildAffine3d(Eigen::Vector3d::Zero(),Eigen::Vector3d(-90*DEG_TO_RAD,0,-90*DEG_TO_RAD-jointAnglesTar(1)),true);
    T_0_Q4 = T_0_Q5*T_Q5_Q4;

    Eigen::Vector3d p_Q4_RCM = T_0_Q4.translation() - RCM.translation();

    Eigen::Vector4d y_FL_h = T_0_FL.matrix() * Eigen::Vector4d(0,1,0,0);
    Eigen::Vector3d y_FL;
    y_FL << y_FL_h(0), y_FL_h(1), y_FL_h(2);

    Eigen::Vector3d z_FL = y_FL.cross(p_Q4_RCM);

    jointAnglesTar(0) = acos(z_FL.dot(x_Q4)/(z_FL.norm()*x_Q4.norm()));

    if(isnan(jointAnglesTar(0)))
    {
        jointAnglesTar(0)  = 0.0;
    }

    if(z_FL.cross(x_Q4).dot(p_Q4_RCM)<0)
    {
        jointAnglesTar(0) = -jointAnglesTar(0);
    }

    Eigen::Affine3d T_Q4_FL = Kinematics::buildAffine3d(Eigen::Vector3d::Zero(),Eigen::Vector3d(-TOOL_PARAMETERS.A_0_Q4*DEG_TO_RAD,-TOOL_PARAMETERS.B_0_Q4*DEG_TO_RAD,-TOOL_PARAMETERS.C_0_Q4*DEG_TO_RAD-jointAnglesTar(0)),true);
    T_Q4_FL.translate(Eigen::Vector3d(-TOOL_PARAMETERS.X_0_Q4,-TOOL_PARAMETERS.Y_0_Q4,-TOOL_PARAMETERS.Z_0_Q4));
    T_0_FL = T_0_Q4*T_Q4_FL;
}
