#include "masterslave/laparoscope.h"

#define DEG_TO_RAD M_PI/180

Laparoscope::Laparoscope(const Eigen::Affine3d startPositionLBR)
{
    toolParameters.A_0_Q4 = 0;
    toolParameters.B_0_Q4 = 90.0;
    toolParameters.C_0_Q4 = 0.0;
    toolParameters.X_0_Q4 = 0.438;
    toolParameters.Y_0_Q4 = 0.0;
    toolParameters.Z_0_Q4 = 0.062;
    toolParameters.L_Q5_Q6 = 0.0088;
    toolParameters.L_Q6_EE = 0.017;

    q4Act=0.0;
    q5Act=0.0;
    q6Act=0.0;
    q5Old=0.0;

    Eigen::Affine3d T_FL_RCM = buildAffine3d(Eigen::Vector3d(0.32,toolParameters.Y_0_Q4,toolParameters.Z_0_Q4),Eigen::Vector3d(toolParameters.A_0_Q4*DEG_TO_RAD,toolParameters.B_0_Q4*DEG_TO_RAD,toolParameters.C_0_Q4*DEG_TO_RAD),true);
    RCM = startPositionLBR*T_FL_RCM;
}

Eigen::Affine3d Laparoscope::buildAffine3d(const Eigen::Vector3d &translXYZ, const Eigen::Vector3d &axisZYX, bool zyx=true)
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

/*void Laparoscope::buildDebugFrameFromTM(const Eigen::Affine3d &T_0_XX, const std::string &name)
{
    tf::TransformBroadcaster br;
    tf::Transform transform;

    tf::transformEigenToTF(T_0_XX, transform);

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", name));
}*/

void Laparoscope::setAngles(double q4, double q5, double q6)
{
    q4Act = q4;
    q5Act = q5;
    q6Act = q6;
    calcDirKin();
}

void Laparoscope::setT_0_EE(Eigen::Affine3d value)
{
    T_0_EE = value;
    calcInvKin();
}

void Laparoscope::calcDirKin()
{
    Eigen::Affine3d T_FL_Q4 = buildAffine3d(Eigen::Vector3d(toolParameters.X_0_Q4,toolParameters.Y_0_Q4,toolParameters.Z_0_Q4),Eigen::Vector3d(toolParameters.A_0_Q4*DEG_TO_RAD,toolParameters.B_0_Q4*DEG_TO_RAD,toolParameters.C_0_Q4*DEG_TO_RAD),true);
    T_FL_Q4.rotate(Eigen::AngleAxis<double>(q4Act,Eigen::Vector3d::UnitX()));
    Eigen::Affine3d T_Q4_Q5 = buildAffine3d(Eigen::Vector3d::Zero(),Eigen::Vector3d(90*DEG_TO_RAD,0,90*DEG_TO_RAD+q5Act),false);
    /* Fehler: Zuerst um Q5 drehen, dann verschieben */
    Eigen::Affine3d T_Q5_Q6 = buildAffine3d(Eigen::Vector3d(toolParameters.L_Q5_Q6,0,0),Eigen::Vector3d(-90*DEG_TO_RAD,0,0),true);
    Eigen::Affine3d T_Q6_EE = buildAffine3d(Eigen::Vector3d::Zero(),Eigen::Vector3d(0,0,q6Act),true);
    T_Q6_EE.translate(Eigen::Vector3d(toolParameters.L_Q6_EE,0,0));

    T_FL_EE = T_FL_Q4*T_Q4_Q5*T_Q5_Q6*T_Q6_EE;
}

void Laparoscope::calcInvKin()
{
    Eigen::Affine3d T_0_Q6 = T_0_EE.translate(Eigen::Vector3d(-toolParameters.L_Q6_EE,0,0));
    Eigen::Vector3d p_RCM_Q6 = T_0_Q6.translation()-RCM.translation();
    Eigen::Vector4d z_Q6_h = T_0_Q6.matrix()*Eigen::Vector4d(0,0,1,0);
    Eigen::Vector3d z_Q6;
    z_Q6 << z_Q6_h(0), z_Q6_h(1), z_Q6_h(2);
    Eigen::Vector3d nPlane = p_RCM_Q6.cross(z_Q6);

    Eigen::Vector4d y_Q6_h = T_0_Q6.matrix()*Eigen::Vector4d(0,-1,0,0);
    Eigen::Vector3d y_Q6;
    y_Q6 << y_Q6_h(0), y_Q6_h(1), y_Q6_h(2);

    q6Tar = acos(nPlane.dot(y_Q6)/(nPlane.norm()*y_Q6.norm()));

    if(isnan(q6Tar))
    {
        q6Tar = 0.0;
    }

    if(q6Tar>M_PI/2)
    {
        q6Tar = M_PI/2;
    }

    if(nPlane.cross(y_Q6).dot(z_Q6)<0)
    {
        q6Tar = -q6Tar;
    }

    Eigen::Affine3d T_EE_Q5 = buildAffine3d(Eigen::Vector3d::Zero(),Eigen::Vector3d(90*DEG_TO_RAD,0,-q6Tar),true);
    T_EE_Q5.translate(Eigen::Vector3d(-toolParameters.L_Q5_Q6,0,0));
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

    Eigen::Vector3d y_Q4 = p_Q5_RCM.cross(z_Q5);

    q5Tar = acos(y_Q4.dot(y_Q5)/(y_Q4.norm()*y_Q5.norm()));

    if(isnan(q5Tar))
    {
        q5Tar = 0.0;
    }

    if(q5Tar>M_PI/2)
    {
        q5Tar = M_PI/2;
    }

    // check if the angle is correct TODO: test!
    if(y_Q4.cross(y_Q5).dot(z_Q5)<0)
    {
        q5Tar = -q5Tar;
    }

    Eigen::Affine3d T_Q5_Q4 = buildAffine3d(Eigen::Vector3d::Zero(),Eigen::Vector3d(-90*DEG_TO_RAD,0,-90*DEG_TO_RAD-q5Tar),true);
    Eigen::Affine3d T_0_Q4 = T_0_Q5*T_Q5_Q4;

    Eigen::Affine3d T_Q4_FL = buildAffine3d(Eigen::Vector3d::Zero(),Eigen::Vector3d(-toolParameters.A_0_Q4*DEG_TO_RAD,-toolParameters.B_0_Q4*DEG_TO_RAD,-toolParameters.C_0_Q4*DEG_TO_RAD),true);
    T_Q4_FL.translate(Eigen::Vector3d(-toolParameters.X_0_Q4,-toolParameters.Y_0_Q4,-toolParameters.Z_0_Q4));
    T_0_FL = T_0_Q4*T_Q4_FL;
    //ROS_INFO_STREAM("Q5: " << q5Tar << " Q6: " << q6Tar << " Q5old: " << q5Old);
    q5Old = q5Tar;

}
