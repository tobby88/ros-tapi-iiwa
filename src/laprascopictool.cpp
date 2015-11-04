#include "masterslave/laprascopictool.h"

#define MM_TO_M 1/1000
#define DEG_TO_RAD M_PI/180

LaprascopicTool::LaprascopicTool(Eigen::Vector3d RCM)
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
{
    Q4 = angles.at(0);
    Q5 = angles.at(1);
    Q6 = angles.at(2);
    calcDirKin();
}

void LaprascopicTool::setQ4(double value)
{
    Q4 = value;
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

    T_0_Q5.setIdentity();
    T_0_Q5.translate(Eigen::Vector3d(toolParameters.X_0_Q4,toolParameters.Y_0_Q4,toolParameters.Z_0_Q4));
    T_0_Q5.rotate(QuaternionFromEuler((Eigen::Vector3d(0,0,Q4)),true));
    T_0_Q5.rotate(QuaternionFromEuler(Eigen::Vector3d(toolParameters.A_0_Q4,toolParameters.B_0_Q4,toolParameters.C_0_Q4),false));

    //direct kinematics in according to Denavit-Hartenberg
    T_Q5_Q6.setIdentity();
    T_Q5_Q6.rotate(QuaternionFromEuler(Eigen::Vector3d(0,0,Q5),true));
    T_Q5_Q6.translate(Eigen::Vector3d(toolParameters.L_Q5_Q6,0,0));
    T_Q5_Q6.rotate(QuaternionFromEuler(Eigen::Vector3d(90*DEG_TO_RAD,0,0),true));

    T_Q6_EE.setIdentity();
    T_Q6_EE.rotate(QuaternionFromEuler(Eigen::Vector3d(0,0,Q6),true));
    T_Q6_EE.translate(Eigen::Vector3d(toolParameters.L_Q6_EE,0,0));    

    T_0_EE = T_0_Q5*T_Q5_Q6*T_Q6_EE;

}

void LaprascopicTool::calcInvKin()
{
    Eigen::Vector3d p_EE = T_0_EE.translation();
    Eigen::Affine3d T_0_Q6 = T_0_EE.translate(Eigen::Vector3d(-toolParameters.L_Q6_EE,0,0));
    // z_Q6 = z_EE
    Eigen::Vector4d z_Q6_h = T_0_Q6.matrix() * Eigen::Vector4d(0,0,1,0);
    Eigen::Vector3d z_Q6;
    z_Q6 << z_Q6_h(0), z_Q6_h(1), z_Q6_h(2);

    Eigen::Vector3d p_EE_RCM = p_EE - RemoteCenterOfMotion;
    Eigen::Vector3d y_Q5 = z_Q6.cross(p_EE_RCM);
    // y_EE = y_Q6
    Eigen::Vector4d y_EE_h = T_0_Q6.matrix() * Eigen::Vector4d(0,1,0,0);
    Eigen::Vector3d y_EE;
    y_EE << y_EE_h(0), y_EE_h(1), y_EE_h(2);
    // Q6
    Q6 = asin(y_Q5.cross(y_EE).norm()/(y_Q5.norm()*y_EE.norm()));
    // Spatprodukt zum Überprüfen: Ist das Volumen kleiner 0 dann ist der Winkel größer als 90° (nicht möglich)
    if(y_Q5.cross(y_EE).dot(z_Q6)<0)
    {
        Q6 = -Q6;
    }
    T_EE_Q5.setIdentity();
    T_EE_Q5.rotate(QuaternionFromEuler(Eigen::Vector3d(0,0,-Q6),true));
    T_EE_Q5.rotate(QuaternionFromEuler(Eigen::Vector3d(90*DEG_TO_RAD,0,0),true));
    T_EE_Q5.translate(Eigen::Vector3d(-toolParameters.L_Q5_Q6,0,0));

    Eigen::Affine3d T_0_Q5 = T_0_EE*T_EE_Q5;
    Eigen::Vector3d p_Q6 = T_0_Q6.translation();
    Eigen::Vector3d p_Q5 = T_0_Q5.translation();

    Eigen::Vector3d p_Q5_RCM = p_Q5 - RemoteCenterOfMotion;
    Eigen::Vector3d p_Q6_Q5 = p_Q6 - p_Q5;

    Q5 = asin(p_Q5_RCM.cross(p_Q6_Q5).norm()/(p_Q5_RCM.norm()*p_Q6_Q5.norm()));
    // Spatprodukt zum Überprüfen: Ist das Volumen kleiner 0 dann ist der Winkel größer als 90° (nicht möglich)
    if(p_Q5_RCM.cross(p_Q6_Q5).dot(y_Q5)<0)
    {
        Q5 = -Q5;
    }
    // TODO: Johann fragen bzgl. Q4
    Q4 = (p_Q5-RemoteCenterOfMotion).norm();
    T_Q5_0.setIdentity();
    T_Q5_0.rotate(QuaternionFromEuler(Eigen::Vector3d(0,0,-Q5),true));
    T_Q5_0.rotate(QuaternionFromEuler(Eigen::Vector3d(-toolParameters.A_0_Q4,-toolParameters.B_0_Q4,-toolParameters.C_0_Q4),true));
    T_Q5_0.rotate(QuaternionFromEuler(Eigen::Vector3d(0,0,-Q4),true));
    T_Q5_0.translate(Eigen::Vector3d(-toolParameters.X_0_Q4,-toolParameters.Y_0_Q4,-toolParameters.Z_0_Q4));




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


