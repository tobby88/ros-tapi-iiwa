#include "masterslave/kinematic/ursulakinematics.h"

UrsulaKinematics::UrsulaKinematics(const Eigen::Affine3d startPositionLBR)
{
    setRCM(startPositionLBR);
    ROS_WARN_STREAM_NAMED("Remote Center of Motion","Position: " << RCM.translation());

}

void UrsulaKinematics::calcDirKin()
{
    //TODO: In DH-Form bringen,

    // LBR Kinematik
    Eigen::Affine3d T_0_Q1 = buildAffine3d(Eigen::Vector3d(0,0,LBR_PARAMETERS.L_0_Q1),Eigen::Vector3d(0,0,lbrJointAnglesAct.Q1),true);
    Eigen::Affine3d T_Q1_Q2 = buildAffine3d(Eigen::Vector3d(0,0,LBR_PARAMETERS.L_Q1_Q2),Eigen::Vector3d(0,lbrJointAnglesAct.Q2,0),true);
    Eigen::Affine3d T_Q2_Q3 = buildAffine3d(Eigen::Vector3d(0,0,LBR_PARAMETERS.L_Q2_Q3),Eigen::Vector3d(0,0,lbrJointAnglesAct.Q3),true);
    Eigen::Affine3d T_Q3_Q4 = buildAffine3d(Eigen::Vector3d(0,0,LBR_PARAMETERS.L_Q3_Q4),Eigen::Vector3d(0,-lbrJointAnglesAct.Q4,0),true);
    Eigen::Affine3d T_Q4_Q5 = buildAffine3d(Eigen::Vector3d(0,0,LBR_PARAMETERS.L_Q4_Q5),Eigen::Vector3d(0,0,lbrJointAnglesAct.Q5),true);
    Eigen::Affine3d T_Q5_Q6 = buildAffine3d(Eigen::Vector3d(0,0,LBR_PARAMETERS.L_Q5_Q6),Eigen::Vector3d(0,lbrJointAnglesAct.Q6,0),true);
    Eigen::Affine3d T_Q6_Q7 = buildAffine3d(Eigen::Vector3d(0,0,LBR_PARAMETERS.L_Q6_Q7),Eigen::Vector3d(0,0,lbrJointAnglesAct.Q7),true);
    Eigen::Affine3d T_Q7_FL = buildAffine3d(Eigen::Vector3d(0,0,LBR_PARAMETERS.L_Q7_FL),Eigen::Vector3d(0,0,0),true);

    T_0_FL = T_0_Q1*T_Q1_Q2*T_Q2_Q3*T_Q3_Q4*T_Q4_Q5*T_Q5_Q6*T_Q6_Q7*T_Q7_FL;

    //Laparoskop Kinematik
    T_FL_EE = calcLaparoscopeDirKin();
    T_0_EE = T_0_FL*T_FL_EE;
}


const lbrDescriptionParameters UrsulaKinematics::LBR_PARAMETERS = { 160, 200, 200, 220, 180, 220, 76, 50};
