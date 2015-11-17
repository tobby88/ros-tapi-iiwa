#include "masterslave/laprascopictool.h"

#define MM_TO_M 1/1000
#define DEG_TO_RAD M_PI/180
#define RAD_TO_DEG 180/M_PI

LaprascopicTool::LaprascopicTool(const Eigen::Affine3d startPoseLBR)
{

    toolParameters.A_0_Q4 = 0;
    toolParameters.B_0_Q4 = 30;
    toolParameters.C_0_Q4 = 0;
    toolParameters.X_0_Q4 = 202.4*MM_TO_M;
    toolParameters.Y_0_Q4 = 0;
    toolParameters.Z_0_Q4 = 481.94*MM_TO_M;
    toolParameters.L_Q5_Q6 = 8.8*MM_TO_M;
    toolParameters.L_Q6_EE = 17*MM_TO_M;
    Q4act=0;
    Q5act=0;
    Q6act=0;
    Eigen::Vector4d Flange_RCM(136*MM_TO_M,0,345*MM_TO_M,1);
    Eigen::Vector4d RCM_h= startPoseLBR.matrix() * Flange_RCM;
    RemoteCenterOfMotion << RCM_h(0), RCM_h(1), RCM_h(2);
}

void LaprascopicTool::buildDebugFrameFromTM(const Eigen::Affine3d &T_0_XX, const std::string &name)
{
    tf::TransformBroadcaster br;
    tf::Transform transform;

    tf::transformEigenToTF(T_0_XX, transform);

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", name));
}

Eigen::Affine3d LaprascopicTool::buildAffine3d(const Eigen::Vector3d &translXYZ, const Eigen::Vector3d &axisZYX, bool zyx=true)
{
    Eigen::Affine3d transl(Eigen::Translation3d(translXYZ(0), translXYZ(1), translXYZ(2)));
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



void LaprascopicTool::setAngles(std::vector<double> angles)
{
    Q4act = angles.at(0);
    Q5act = angles.at(1);
    Q6act = angles.at(2);
    calcDirKin();
}

void LaprascopicTool::setQ4(double value)
{
    Q4act = value;
    calcDirKin();
}

void LaprascopicTool::setQ5(double value)
{
    Q5act = value;
    calcDirKin();
}

void LaprascopicTool::setQ6(double value)
{
    Q6act = value;
    calcDirKin();
}

void LaprascopicTool::setT_0_EE(Eigen::Affine3d value)
{
    T_0_EE = value;
    calcInvKin();
}

void LaprascopicTool::calcDirKin()
{
    T_FL_Q4 = buildAffine3d(Eigen::Vector3d(toolParameters.X_0_Q4,toolParameters.Y_0_Q4,toolParameters.Z_0_Q4),Eigen::Vector3d(toolParameters.A_0_Q4*DEG_TO_RAD,toolParameters.B_0_Q4*DEG_TO_RAD,toolParameters.C_0_Q4*DEG_TO_RAD),true);
    T_FL_Q4.rotate(Eigen::AngleAxis<double>(Q4act,Eigen::Vector3d::UnitZ()));
    //Q4 um Z
    buildDebugFrameFromTM(T_FL_Q4,"TF_FL_Q4");
    //direct kinematics in according to Denavit-Hartenberg
    T_Q4_Q5 = buildAffine3d(Eigen::Vector3d(0,0,0),Eigen::Vector3d(90*DEG_TO_RAD,0,90*DEG_TO_RAD),false);
    T_Q4_Q5.rotate(Eigen::AngleAxis<double>(Q5act,Eigen::Vector3d::UnitZ()));
    //Q5 um z
    buildDebugFrameFromTM(T_Q4_Q5,"TF_Q4_Q5");


    T_Q5_EE = buildAffine3d(Eigen::Vector3d(toolParameters.L_Q5_Q6,0,0),Eigen::Vector3d(-90*DEG_TO_RAD,0,0),false);
    // Q6 um z
    T_Q5_EE.rotate(QuaternionFromEuler(Eigen::Vector3d(0,0,Q6act),true));
    T_Q5_EE.translate(toolParameters.L_Q6_EE*Eigen::Vector3d::UnitX());

    T_FL_EE = T_FL_Q4*T_Q4_Q5*T_Q5_EE;
    buildDebugFrameFromTM(T_FL_EE,"TF_FL_EE");

}

void LaprascopicTool::calcInvKin()
{


    T_0_EE = T_0_EE;
    buildDebugFrameFromTM(T_0_EE,"T_0_EE");
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
    Q6tar = asin(y_Q5.cross(y_EE).norm()/(y_Q5.norm()*y_EE.norm()));
    // Spatprodukt zum Überprüfen: Ist das Volumen kleiner 0 dann ist der Winkel größer als 90° (nicht möglich)
    if(y_Q5.cross(y_EE).dot(z_Q6)<0)
    {
        Q6tar = -Q6tar;
    }
    Eigen::Affine3d T_EE_Q5;
    //ROS_INFO("Q6tar: %f",Q6tar);
    T_EE_Q5.setIdentity();
    T_EE_Q5.translate(Eigen::Vector3d(-toolParameters.L_Q6_EE,0,0));
    T_EE_Q5.rotate(QuaternionFromEuler(Eigen::Vector3d(0,0,-Q6tar),true));
    T_EE_Q5.rotate(QuaternionFromEuler(Eigen::Vector3d(90*DEG_TO_RAD,0,0),false));
    T_EE_Q5.translate(Eigen::Vector3d(-toolParameters.L_Q5_Q6,0,0));

    Eigen::Affine3d T_0_Q5 = T_0_EE*T_EE_Q5;
    buildDebugFrameFromTM(T_0_Q5,"T_0_Q5");

    Eigen::Vector3d p_Q6 = T_0_Q6.translation();
    Eigen::Vector3d p_Q5 = T_0_Q5.translation();

    Eigen::Vector3d p_Q5_RCM = p_Q5 - RemoteCenterOfMotion;
    Eigen::Vector3d p_Q6_Q5 = p_Q6 - p_Q5;

    Q5tar = asin(p_Q5_RCM.cross(p_Q6_Q5).norm()/(p_Q5_RCM.norm()*p_Q6_Q5.norm()));
    // Spatprodukt zum Überprüfen: Ist das Volumen kleiner 0 dann ist der Winkel größer als 90° (nicht möglich)
    if(p_Q5_RCM.cross(p_Q6_Q5).dot(y_Q5)<0)
    {
        Q5tar = -Q5tar;
    }

    // TODO: Johann fragen bzgl. Q4
    Q4tar = (p_Q5-RemoteCenterOfMotion).norm();
    Eigen::Affine3d T_Q5_Q4;
    T_Q5_Q4.setIdentity();
    T_Q5_Q4.rotate(QuaternionFromEuler(Eigen::Vector3d(0,0,-Q5tar),true));
    T_Q5_Q4.rotate(QuaternionFromEuler(Eigen::Vector3d(-90*DEG_TO_RAD,0,-90*DEG_TO_RAD),true));

    Eigen::Affine3d T_0_Q4 = T_0_Q5*T_Q5_Q4;
    Eigen::Affine3d T_Q4_FL;
    T_Q4_FL.setIdentity();
    //T_Q4_FL.rotate(QuaternionFromEuler(Eigen::Vector3d(0,0,-Q4tar),true));
    T_Q4_FL.rotate(QuaternionFromEuler(Eigen::Vector3d(-toolParameters.A_0_Q4*DEG_TO_RAD,-toolParameters.B_0_Q4*DEG_TO_RAD,-toolParameters.C_0_Q4*DEG_TO_RAD),false));

    T_Q4_FL.translate(Eigen::Vector3d(-toolParameters.X_0_Q4,-toolParameters.Y_0_Q4,-toolParameters.Z_0_Q4));

    T_0_FL = T_0_Q4*T_Q4_FL;
    //T_0_FL.setIdentity();
    buildDebugFrameFromTM(T_0_FL,"T_0_FL");
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



