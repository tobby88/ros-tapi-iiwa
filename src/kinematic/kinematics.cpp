#include "masterslave/kinematic/kinematics.h"

#define DEG_TO_RAD M_PI/180

Eigen::Affine3d Kinematics::buildAffine3d(const Eigen::Vector3d &translXYZ, const Eigen::Vector3d &axisZYX, bool zyx=true)
{
    Eigen::Affine3d transl;
    transl.setIdentity();
    transl.translate(translXYZ);
    if(zyx)
    {
        transl.rotate(Eigen::AngleAxis<double>(axisZYX(2), Eigen::Vector3d::UnitZ()));
        transl.rotate(Eigen::AngleAxis<double>(axisZYX(1), Eigen::Vector3d::UnitY()));
        transl.rotate(Eigen::AngleAxis<double>(axisZYX(0), Eigen::Vector3d::UnitX()));
    }
    else
    {
        transl.rotate(Eigen::AngleAxis<double>(axisZYX(0), Eigen::Vector3d::UnitX()));
        transl.rotate(Eigen::AngleAxis<double>(axisZYX(1), Eigen::Vector3d::UnitY()));
        transl.rotate(Eigen::AngleAxis<double>(axisZYX(2), Eigen::Vector3d::UnitZ()));
    }
    return transl;
}

void Kinematics::setT_0_EE(Eigen::Affine3d value)
{
    T_0_EE = value;
    calcInvKin();
}

void Kinematics::setRCM(const Eigen::Affine3d startPose)
{
    Eigen::Affine3d T_FL_RCM = buildAffine3d(Eigen::Vector3d(TOOL_PARAMETERS.X_RCM,TOOL_PARAMETERS.Y_0_Q4,TOOL_PARAMETERS.Z_0_Q4),Eigen::Vector3d(TOOL_PARAMETERS.A_0_Q4*DEG_TO_RAD,TOOL_PARAMETERS.B_0_Q4*DEG_TO_RAD,TOOL_PARAMETERS.C_0_Q4*DEG_TO_RAD),true);
    RCM = startPose*T_FL_RCM;
}

Eigen::Affine3d Kinematics::calcLaparoscopeDirKin()
{
    T_FL_Q4 = buildAffine3d(Eigen::Vector3d(TOOL_PARAMETERS.X_0_Q4,TOOL_PARAMETERS.Y_0_Q4,TOOL_PARAMETERS.Z_0_Q4),Eigen::Vector3d(TOOL_PARAMETERS.A_0_Q4*DEG_TO_RAD,TOOL_PARAMETERS.B_0_Q4*DEG_TO_RAD,TOOL_PARAMETERS.C_0_Q4*DEG_TO_RAD),true);
    T_FL_Q4.rotate(Eigen::AngleAxis<double>(toolAnglesAct.Q4,Eigen::Vector3d::UnitZ()));
    Eigen::Affine3d T_Q4_Q5 = buildAffine3d(Eigen::Vector3d::Zero(),Eigen::Vector3d(90*DEG_TO_RAD,0,90*DEG_TO_RAD+toolAnglesAct.Q5),false);
    /* Fehler: Zuerst um Q5 drehen, dann verschieben */
    Eigen::Affine3d T_Q5_Q6 = buildAffine3d(Eigen::Vector3d(TOOL_PARAMETERS.L_Q5_Q6,0,0),Eigen::Vector3d(-90*DEG_TO_RAD,0,0),true);
    Eigen::Affine3d T_Q6_EE = buildAffine3d(Eigen::Vector3d::Zero(),Eigen::Vector3d(0,0,toolAnglesAct.Q6),true);
    T_Q6_EE.translate(Eigen::Vector3d(TOOL_PARAMETERS.L_Q6_EE,0,0));

    T_FL_Q5 = T_FL_Q4*T_Q4_Q5;
    T_FL_Q6 = T_FL_Q5*T_Q5_Q6;

    return T_FL_Q4*T_Q4_Q5*T_Q5_Q6*T_Q6_EE;
}


const toolDescriptionParameters Kinematics::TOOL_PARAMETERS = {0.438, 0.0, 0.062, 0.0, 90.0, 0.0, 0.0088, 0.017, 0.305};

