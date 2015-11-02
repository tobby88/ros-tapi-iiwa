#include "masterslave/laprascopictool.h"

#define MM_TO_M 1/1000
#define DEG_TO_RAD M_PI/180

LaprascopicTool::LaprascopicTool(Eigen::Vector4d RCM)
{
    RemoteCenterOfMotion = RCM;
    toolParameters.A_0_Q4 = -90*DEG_TO_RAD;
    toolParameters.B_0_Q4 = 0*DEG_TO_RAD;
    toolParameters.C_0_Q4 = 60*DEG_TO_RAD;
    toolParameters.X_0_Q4 = -202.4*MM_TO_M;
    toolParameters.Y_0_Q4 = 0;
    toolParameters.Z_0_Q4 = 481.94*MM_TO_M;
    toolParameters.L_Q5_Q6 = 8.8*MM_TO_M;
    toolParameters.L_Q6_EE = 17*MM_TO_M;

}

void LaprascopicTool::setAngles(std::vector<double> angles)
{;
    Q5 = angles.at(1);
    Q6 = angles.at(2);
    calcDirKin();
}

void LaprascopicTool::setQ5(double value)
{
    Q5 = value;
    calcDirKin();
}

void LaprascopicTool::setQ6(double value)
{
    Q6 = value;
    calcDirKin();
}

void LaprascopicTool::setT_0_EE(Eigen::Affine3d value)
{
    T_0_EE = value;
    calcInvKin();
}

void LaprascopicTool::calcDirKin()
{

    T_0_Q5.Identity();
    T_0_Q5.translate(Eigen::Vector3d(toolParameters.X_0_Q4,toolParameters.Y_0_Q4,toolParameters.Z_0_Q4));
    T_0_Q5.rotate(QuaternionFromEuler(Eigen::Vector3d(toolParameters.A_0_Q4,toolParameters.B_0_Q4,toolParameters.C_0_Q4),true));
    //direct kinematics in according to Denavit-Hartenberg
    T_Q5_Q6.Identity();
    T_Q5_Q6.rotate(QuaternionFromEuler(Eigen::Vector3d(0,0,Q5*DEG_TO_RAD),true));
    T_Q5_Q6.translate(Eigen::Vector3d(toolParameters.L_Q5_Q6,0,0));
    T_Q5_Q6.rotate(QuaternionFromEuler(Eigen::Vector3d(90*DEG_TO_RAD,0,0),true));

    T_Q6_EE.Identity();
    T_Q6_EE.rotate(QuaternionFromEuler(Eigen::Vector3d(0,0,Q6),true));
    T_Q6_EE.translate(Eigen::Vector3d(toolParameters.L_Q6_EE,0,0));
    T_Q6_EE.rotate(QuaternionFromEuler(Eigen::Vector3d(0,-90*DEG_TO_RAD,0),true));

    T_0_EE = T_0_Q5*T_Q5_Q6*T_Q6_EE;

}

void LaprascopicTool::calcInvKin(){


}

Eigen::Quaternion<double> LaprascopicTool::QuaternionFromEuler(const Eigen::Vector3d &eulerXYZ, bool ZYX=true)

{
    Eigen::Quaternion<double> quat;
    quat.Identity();
    Eigen::AngleAxisd zAngle(eulerXYZ[2], Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yAngle(eulerXYZ[1], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd xAngle(eulerXYZ[0], Eigen::Vector3d::UnitX());
    if(ZYX)
        quat = zAngle * yAngle * xAngle;
    else
        quat = xAngle * yAngle * zAngle;

    return quat;
}

int main(){}


