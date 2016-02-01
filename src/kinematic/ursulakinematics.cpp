#include "masterslave/kinematic/ursulakinematics.h"

UrsulaKinematics::UrsulaKinematics(const Eigen::Affine3d startPositionLBR)
{
    setRCM(startPositionLBR);
    ROS_WARN_STREAM_NAMED("Remote Center of Motion","Position: " << RCM.translation());
    isDirKinCalced = false;
}

void UrsulaKinematics::calcDirKin()
{
    //TODO: In DH-Form bringen,

    // LBR Kinematik
    T_0_Q1 = buildAffine3d(Eigen::Vector3d(0,0,LBR_PARAMETERS.L_0_Q1),Eigen::Vector3d(0,0,lbrJointAnglesAct.Q1),true);
    Eigen::Affine3d T_Q1_Q2 = buildAffine3d(Eigen::Vector3d(0,0,LBR_PARAMETERS.L_Q1_Q2),Eigen::Vector3d(0,lbrJointAnglesAct.Q2,0),true);
    Eigen::Affine3d T_Q2_Q3 = buildAffine3d(Eigen::Vector3d(0,0,LBR_PARAMETERS.L_Q2_Q3),Eigen::Vector3d(0,0,lbrJointAnglesAct.Q3),true);
    Eigen::Affine3d T_Q3_Q4 = buildAffine3d(Eigen::Vector3d(0,0,LBR_PARAMETERS.L_Q3_Q4),Eigen::Vector3d(0,-lbrJointAnglesAct.Q4,0),true);
    Eigen::Affine3d T_Q4_Q5 = buildAffine3d(Eigen::Vector3d(0,0,LBR_PARAMETERS.L_Q4_Q5),Eigen::Vector3d(0,0,lbrJointAnglesAct.Q5),true);
    Eigen::Affine3d T_Q5_Q6 = buildAffine3d(Eigen::Vector3d(0,0,LBR_PARAMETERS.L_Q5_Q6),Eigen::Vector3d(0,lbrJointAnglesAct.Q6,0),true);
    Eigen::Affine3d T_Q6_Q7 = buildAffine3d(Eigen::Vector3d(0,0,LBR_PARAMETERS.L_Q6_Q7),Eigen::Vector3d(0,0,lbrJointAnglesAct.Q7),true);
    Eigen::Affine3d T_Q7_FL = buildAffine3d(Eigen::Vector3d(0,0,LBR_PARAMETERS.L_Q7_FL),Eigen::Vector3d(0,0,0),true);

    // important for jacobi matrix
    T_0_Q2 = T_0_Q1*T_Q1_Q2;
    T_0_Q3 = T_0_Q2*T_Q2_Q3;
    T_0_Q4 = T_0_Q3*T_Q3_Q4;
    T_0_Q5 = T_0_Q4*T_Q4_Q5;
    T_0_Q6 = T_0_Q5*T_Q5_Q6;
    T_0_Q7 = T_0_Q6*T_Q6_Q7;

    T_0_FL = T_0_Q7*T_Q7_FL;

    //Laparoskop Kinematik
    T_FL_EE = calcLaparoscopeDirKin();

    T_0_EE = T_0_FL*T_FL_EE;

    isDirKinCalced = true;
}

void UrsulaKinematics::calcInvKin()
{

}

void UrsulaKinematics::calcAnalyticalJacobian()
{
    if(!isDirKinCalced) return;

    Eigen::Affine3d T_0_Q4Lap = T_0_FL*T_FL_Q4;
    Eigen::Affine3d T_0_Q5Lap = T_0_FL*T_FL_Q5;
    Eigen::Affine3d T_0_Q6Lap = T_0_FL*T_FL_Q6;

    //z-Axis of each joint
    Eigen::Vector3d z_Q1 = T_0_Q1.matrix().col(3).head(3);
    Eigen::Vector3d z_Q2 = T_0_Q2.matrix().col(3).head(3);
    Eigen::Vector3d z_Q3 = T_0_Q3.matrix().col(3).head(3);
    Eigen::Vector3d z_Q4 = T_0_Q4.matrix().col(3).head(3);
    Eigen::Vector3d z_Q5 = T_0_Q5.matrix().col(3).head(3);
    Eigen::Vector3d z_Q6 = T_0_Q6.matrix().col(3).head(3);
    Eigen::Vector3d z_Q7 = T_0_Q7.matrix().col(3).head(3);
    Eigen::Vector3d z_Q8 = T_0_Q4Lap.matrix().col(3).head(3);
    Eigen::Vector3d z_Q9 = T_0_Q5Lap.matrix().col(3).head(3);
    Eigen::Vector3d z_Q10 = T_0_Q6Lap.matrix().col(3).head(3);

    Eigen::Vector3d x_1_EE = T_0_EE.translation() - T_0_Q1.translation();
    Eigen::Vector3d x_2_EE = T_0_EE.translation() - T_0_Q2.translation();
    Eigen::Vector3d x_3_EE = T_0_EE.translation() - T_0_Q3.translation();
    Eigen::Vector3d x_4_EE = T_0_EE.translation() - T_0_Q4.translation();
    Eigen::Vector3d x_5_EE = T_0_EE.translation() - T_0_Q5.translation();
    Eigen::Vector3d x_6_EE = T_0_EE.translation() - T_0_Q6.translation();
    Eigen::Vector3d x_7_EE = T_0_EE.translation() - T_0_Q7.translation();
    Eigen::Vector3d x_8_EE = T_0_EE.translation() - T_0_Q4Lap.translation();
    Eigen::Vector3d x_9_EE = T_0_EE.translation() - T_0_Q5Lap.translation();
    Eigen::Vector3d x_10_EE = T_0_EE.translation() - T_0_Q6Lap.translation();

    Eigen::Vector3d zr1 = z_Q1.cross(x_1_EE);
    Eigen::Vector3d zr2 = z_Q2.cross(x_2_EE);
    Eigen::Vector3d zr3 = z_Q3.cross(x_3_EE);
    Eigen::Vector3d zr4 = z_Q4.cross(x_4_EE);
    Eigen::Vector3d zr5 = z_Q5.cross(x_5_EE);
    Eigen::Vector3d zr6 = z_Q6.cross(x_6_EE);
    Eigen::Vector3d zr7 = z_Q7.cross(x_7_EE);
    Eigen::Vector3d zr8 = z_Q8.cross(x_8_EE);
    Eigen::Vector3d zr9 = z_Q9.cross(x_9_EE);
    Eigen::Vector3d zr10 = z_Q10.cross(x_10_EE);

    geomJacobian(6,10);
    geomJacobian.row(0) << zr1, z_Q1;
    geomJacobian.row(1) << zr2, z_Q2;
    geomJacobian.row(2) << zr3, z_Q3;
    geomJacobian.row(3) << zr4, z_Q4;
    geomJacobian.row(4) << zr5, z_Q5;
    geomJacobian.row(5) << zr6, z_Q6;
    geomJacobian.row(6) << zr7, z_Q7;
    geomJacobian.row(7) << zr8, z_Q8;
    geomJacobian.row(8) << zr9, z_Q9;
    geomJacobian.row(9) << zr10, z_Q10;

    Eigen::MatrixXd omega(6,6);

    omega.topLeftCorner(3,3) = Eigen::MatrixXd::Identity(3,3);
    omega.topRightCorner(3,3) = Eigen::MatrixXd::Zero(3,3);
    omega.bottomLeftCorner(3,3) = Eigen::MatrixXd::Zero(3,3);

    //Transformation from geometrical Jacobian to analytical Jacobian
    double alpha = atan2(T_0_EE.matrix()(2,1),T_0_EE.matrix()(2,2));
    double gamma = atan2(T_0_EE.matrix()(1,0),T_0_EE.matrix()(0,0));
    double beta = atan2(-T_0_EE.matrix()(2,0),T_0_EE.matrix()(0,0)*cos(gamma)+T_0_EE.matrix()(1,0)*sin(gamma));

    Eigen::Matrix3d omegaKard;
    omegaKard.row(0) << cos(beta)*cos(gamma), -sin(gamma), 0;
    omegaKard.row(1) << sin(gamma)*cos(beta), cos(gamma), 0;
    omegaKard.row(2) << -sin(beta), 0, 1;

    omega.bottomRightCorner(3,3) = omegaKard;

    analyticalJacobian(6,10);
    analyticalJacobian = omega.inverse()*geomJacobian;




}


const lbrDescriptionParameters UrsulaKinematics::LBR_PARAMETERS = { 160, 200, 200, 220, 180, 220, 76, 50};
//in according to LBR Specifications (in Degree)
const lbrJointAngles UrsulaKinematics::LBR_MAX_ANGLES = { 170, 120, 170, 120, 170, 120, 175};
const lbrJointAngles UrsulaKinematics::LBR_MIN_ANGLES = {-170, -120, -170, -120, -170, -120, -175};
//in according to LBR Specifications (in Degree per Second)
const lbrJointAngles UrsulaKinematics::LBR_MAX_ANGLES_SPEED = { 85, 85, 100, 75, 130, 135, 135};
const lbrJointAngles UrsulaKinematics::LBR_MIN_ANGLES_SPEED = { -85, -85, -100, -75, -130, -135, -135};
