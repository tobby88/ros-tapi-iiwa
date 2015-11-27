#include "masterslave/laprascopictool.h"

#define DEG_TO_RAD M_PI/180

LaprascopicTool::LaprascopicTool(const Eigen::Affine3d startPoseLBR)
{

    toolParameters.A_0_Q4 = 0;
    toolParameters.B_0_Q4 = 90.0;
    toolParameters.C_0_Q4 = 0.0;
    toolParameters.X_0_Q4 = 0.438;
    toolParameters.Y_0_Q4 = 0.0;
    toolParameters.Z_0_Q4 = 0.062;
    toolParameters.L_Q5_Q6 = 0.0088;
    toolParameters.L_Q6_EE = 0.017;
    Q4act=0.0;
    Q5act=0.0;
    Q6act=0.0;
    Q5old=0.0;
    Eigen::Affine3d T_FL_RCM = buildAffine3d(Eigen::Vector3d(0.315,toolParameters.Y_0_Q4,toolParameters.Z_0_Q4),Eigen::Vector3d(toolParameters.A_0_Q4*DEG_TO_RAD,toolParameters.B_0_Q4*DEG_TO_RAD,toolParameters.C_0_Q4*DEG_TO_RAD),true);
    RemoteCenterOfMotion = (startPoseLBR * T_FL_RCM);
    T_0_FL = startPoseLBR;
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



void LaprascopicTool::setAngles(double Q4, double Q5, double Q6)
{
    Q4act = Q4;
    Q5act = Q5;
    Q6act = Q6;
    calcDirKin();
}

void LaprascopicTool::setQ4(double value)
{
    Q4act = value;
    calcDirKin();
}

void LaprascopicTool::setQ5(double value)
{
    Q5act = value+0.0001;
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
    T_FL_Q4.setIdentity();
    T_FL_Q4.translate(Eigen::Vector3d(toolParameters.X_0_Q4,toolParameters.Y_0_Q4,toolParameters.Z_0_Q4));
    T_FL_Q4.rotate(QuaternionFromEuler(Eigen::Vector3d(toolParameters.A_0_Q4*DEG_TO_RAD,toolParameters.B_0_Q4*DEG_TO_RAD,toolParameters.C_0_Q4*DEG_TO_RAD),true));
    //T_FL_Q4.rotate(Eigen::AngleAxis<double>(Q4act,Eigen::Vector3d::UnitZ()));
    //Q4 um Z
    buildDebugFrameFromTM(T_FL_Q4,"TF_FL_Q4");
    //direct kinematics in according to Denavit-Hartenberg
    T_Q4_Q5.setIdentity();
    T_Q4_Q5.rotate(QuaternionFromEuler(Eigen::Vector3d(90*DEG_TO_RAD,0,90*DEG_TO_RAD),false));
    T_Q4_Q5.rotate(Eigen::AngleAxis<double>(Q5act,Eigen::Vector3d::UnitZ()));
    //Q5 um z
    buildDebugFrameFromTM(T_Q4_Q5,"TF_Q4_Q5");


    T_Q5_Q6.setIdentity();
    T_Q5_Q6.translate(Eigen::Vector3d(toolParameters.L_Q5_Q6,0,0));
    T_Q5_Q6.rotate(QuaternionFromEuler(Eigen::Vector3d(-90*DEG_TO_RAD,0,0),false));
    // Q6 um z
    T_Q6_EE.setIdentity();
    T_Q6_EE.rotate(QuaternionFromEuler(Eigen::Vector3d(0,0,Q6act),true));
    T_Q6_EE.translate(toolParameters.L_Q6_EE*Eigen::Vector3d::UnitX());

    T_FL_EE = T_FL_Q4*T_Q4_Q5*T_Q5_Q6*T_Q6_EE;
    buildDebugFrameFromTM(T_FL_EE,"TF_FL_EE");

}

void LaprascopicTool::calcInvKin()
{
    Eigen::Vector3d p_EE = T_0_EE.translation()-RemoteCenterOfMotion.translation();
    buildDebugFrameFromTM(T_0_EE,"T_0_EE");
    Eigen::Affine3d T_0_Q6 = T_0_EE.translate(Eigen::Vector3d(-toolParameters.L_Q6_EE,0,0));
    Eigen::Vector3d p_Q6 =  T_0_Q6.translation()-RemoteCenterOfMotion.translation();

    Eigen::Vector4d z_Q6_h = T_0_Q6.matrix() * Eigen::Vector4d(0,0,1,0);
    Eigen::Vector3d z_Q6;
    z_Q6 << z_Q6_h(0), z_Q6_h(1), z_Q6_h(2);

    Eigen::Vector4d y_Q6_h = T_0_Q6.matrix() * Eigen::Vector4d(0,-1,0,0);
    Eigen::Vector3d y_Q6;
    y_Q6 << y_Q6_h(0), y_Q6_h(1), y_Q6_h(2);

    Eigen::Vector3d nPlane;
    if(p_Q6.normalized()==z_Q6.normalized() || p_Q6.normalized() == -z_Q6.normalized())
    {
        nPlane = y_Q6;
    }
    else
    {
        nPlane = p_Q6.cross(z_Q6);
    }

    Eigen::Vector3d p_EE_RCM = p_EE - RemoteCenterOfMotion.translation();
    //siehe V-Rep
    // y_EE = y_Q6

    // Q6
    Q6tar = acos(nPlane.dot(y_Q6)/(nPlane.norm()*y_Q6.norm()));
    // Spatprodukt zum Überprüfen: Ist das Volumen kleiner 0 dann ist der Winkel größer als 90° (nicht möglich)
    if(nPlane.cross(y_Q6).dot(z_Q6)<0)
    {
        Q6tar = -Q6tar;
    }
    if(isnan(Q6tar))
    {
        Q6tar = 0.0;
    }
    Eigen::Affine3d T_EE_Q6;
    ROS_INFO("Q6tar: %f",Q6tar);
    T_EE_Q6.setIdentity();
    //T_EE_Q6.translate(Eigen::Vector3d(-toolParameters.L_Q6_EE,0,0));
    T_EE_Q6.rotate(QuaternionFromEuler(Eigen::Vector3d(0,0,-Q6tar),true));
    T_EE_Q6.rotate(QuaternionFromEuler(Eigen::Vector3d(90*DEG_TO_RAD,0,0),false));
    Eigen::Affine3d T_Q6_Q5;
    T_Q6_Q5.setIdentity();
    T_Q6_Q5.translate(Eigen::Vector3d(-toolParameters.L_Q5_Q6,0,0));

    Eigen::Affine3d T_0_Q5 = T_0_EE*T_EE_Q6*T_Q6_Q5;
    buildDebugFrameFromTM(T_0_Q5,"T_0_Q5");

    Eigen::Vector3d p_Q5 = T_0_Q5.translation() - RemoteCenterOfMotion.translation();

    buildDebugFrameFromTM(RemoteCenterOfMotion,"RCM");
    Eigen::Vector4d RCM_x_h = RemoteCenterOfMotion.matrix()*Eigen::Vector4d(0,0,1,0);
    Eigen::Vector3d RCM_x;
    RCM_x << RCM_x_h(0), RCM_x_h(1), RCM_x_h(2);

    Eigen::Vector4d x_Q5_h = T_0_Q5.matrix() * Eigen::Vector4d(1,0,0,0);
    Eigen::Vector3d x_Q5;
    x_Q5 << x_Q5_h(0), x_Q5_h(1), x_Q5_h(2);

    Eigen::Vector4d y_Q5_h = T_0_Q5.matrix() * Eigen::Vector4d(0,0,1,0);
    Eigen::Vector3d y_Q5;
    y_Q5 << y_Q5_h(0), y_Q5_h(1), y_Q5_h(2);

    Q5tar = acos(RCM_x.dot(x_Q5)/(RCM_x.norm()*x_Q5.norm()));

    ROS_INFO_STREAM("Spatprodukt: " << RCM_x.cross(x_Q5).dot(y_Q5));

    if(RCM_x.cross(x_Q5).dot(y_Q5)<0)
    {
        Q5tar = -Q5tar;
    }

    if(isnan(Q5tar))
    {
        Q5tar = 0.0;
    }


    ROS_INFO("Q5_tar: %f",Q5tar);
    // TODO: Johann fragen bzgl. Q4
    //Q4tar = (p_Q5-RemoteCenterOfMotion).norm();
    Eigen::Affine3d T_Q5_Q4;
    T_Q5_Q4.setIdentity();
    T_Q5_Q4.rotate(QuaternionFromEuler(Eigen::Vector3d(0,0,-Q5tar),true));
    T_Q5_Q4.rotate(QuaternionFromEuler(Eigen::Vector3d(-90*DEG_TO_RAD,0,-90*DEG_TO_RAD),true));

    Eigen::Affine3d T_0_Q4 = T_0_Q5*T_Q5_Q4;
    Eigen::Affine3d T_Q4_FL;
    T_Q4_FL.setIdentity();
    //T_Q4_FL.rotate(QuaternionFromEuler(Eigen::Vector3d(0,0,-Q4tar),true));
    T_Q4_FL.rotate(QuaternionFromEuler(Eigen::Vector3d(-toolParameters.A_0_Q4*DEG_TO_RAD,-toolParameters.B_0_Q4*DEG_TO_RAD,-toolParameters.C_0_Q4*DEG_TO_RAD),false));
    Eigen::Affine3d T_FL;
    T_FL.setIdentity();
    T_FL.translate(Eigen::Vector3d(-toolParameters.X_0_Q4,-toolParameters.Y_0_Q4,-toolParameters.Z_0_Q4));

    T_0_FL = T_0_Q4*T_Q4_FL*T_FL;
    buildDebugFrameFromTM(T_0_FL,"T_0_FL");
    // round values

    Q5old = Q5tar;
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



