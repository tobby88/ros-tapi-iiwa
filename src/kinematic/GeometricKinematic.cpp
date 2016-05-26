#include "masterslave/kinematic/GeometricKinematic.h"


GeometricKinematic::GeometricKinematic(ros::NodeHandle& nh): nh_(nh)
{
    jointAnglesAct = Eigen::VectorXd(3);
    jointAnglesTar = Eigen::VectorXd(3);

    rcmServer = nh_.advertiseService("/RCM",&GeometricKinematic::rcmCallback,this);
    directKinematicsServer = nh_.advertiseService("/directKinematics",&GeometricKinematic::directKinematicsCallback,this);
    inverseKinematicsServer = nh_.advertiseService("/inverseKinematics",&GeometricKinematic::inverseKinematicsCallback,this);

    ros::spin();
}

bool GeometricKinematic::rcmCallback(masterslave::GeometricKinematicRCM::Request &req, masterslave::GeometricKinematicRCM::Response &resp)
{
    Eigen::Affine3d T_0_FL;
    tf::poseMsgToEigen(req.T_0_FL,T_0_FL);
    RCM = T_0_FL * buildAffine3d(Eigen::Vector3d(TOOL_PARAMETERS.X_RCM,TOOL_PARAMETERS.Y_0_Q4,TOOL_PARAMETERS.Z_0_Q4),Eigen::Vector3d(TOOL_PARAMETERS.A_0_Q4*DEG_TO_RAD,TOOL_PARAMETERS.B_0_Q4*DEG_TO_RAD,TOOL_PARAMETERS.C_0_Q4*DEG_TO_RAD),true);
    tf::poseEigenToMsg(RCM,resp.trocar);
    ROS_WARN_STREAM(RCM.translation());
    return true;
}

bool GeometricKinematic::directKinematicsCallback(masterslave::GeometricKinematicDirectKinematics::Request &req, masterslave::GeometricKinematicDirectKinematics::Response &resp)
{
    Eigen::Affine3d T_0_FL;
    tf::poseMsgToEigen(req.T_0_FL,T_0_FL);
    Eigen::VectorXd jointAngles = Eigen::VectorXd::Map(req.jointAngles.data(),req.jointAngles.size());
    T_0_EE = T_0_FL * calcDirKin(jointAngles);
    tf::poseEigenToMsg(T_0_EE,resp.T_0_EE);
    ROS_INFO_STREAM("TCP: \n" << T_0_EE.matrix());
    return true;
}

bool GeometricKinematic::inverseKinematicsCallback(masterslave::GeometricKinematicInverseKinematics::Request &req, masterslave::GeometricKinematicInverseKinematics::Response &resp)
{
    Eigen::Affine3d desEEPosition;
    tf::poseMsgToEigen(req.T_0_EE,desEEPosition);
    calcInvKin(desEEPosition);
    tf::poseEigenToMsg(T_0_FL,resp.T_0_FL);
    std::vector<double> jointAnglesTarget(jointAnglesTar.data(),jointAnglesTar.data()+jointAnglesTar.rows()*jointAnglesTar.cols());
    resp.jointAnglesTarget = jointAnglesTarget;
    return true;
}

Eigen::Affine3d GeometricKinematic::calcDirKin(Eigen::VectorXd jointAngles)
{
    T_FL_Q4 = buildAffine3d(Eigen::Vector3d(TOOL_PARAMETERS.X_0_Q4,TOOL_PARAMETERS.Y_0_Q4,TOOL_PARAMETERS.Z_0_Q4),Eigen::Vector3d(TOOL_PARAMETERS.A_0_Q4*DEG_TO_RAD,TOOL_PARAMETERS.B_0_Q4*DEG_TO_RAD,TOOL_PARAMETERS.C_0_Q4*DEG_TO_RAD),true);
    T_FL_Q4.rotate(Eigen::AngleAxis<double>(jointAngles(0),Eigen::Vector3d::UnitZ()));
    Eigen::Affine3d T_Q4_Q5 = buildAffine3d(Eigen::Vector3d::Zero(),Eigen::Vector3d(90*DEG_TO_RAD,0,90*DEG_TO_RAD+jointAngles(1)),false);
    /* Fehler: Zuerst um Q5 drehen, dann verschieben */
    Eigen::Affine3d T_Q5_Q6 = buildAffine3d(Eigen::Vector3d(TOOL_PARAMETERS.L_Q5_Q6,0,0),Eigen::Vector3d(-90*DEG_TO_RAD,0,0),true);
    Eigen::Affine3d T_Q6_EE = buildAffine3d(Eigen::Vector3d::Zero(),Eigen::Vector3d(0,0,jointAngles(2)),true);
    T_Q6_EE.translate(Eigen::Vector3d(TOOL_PARAMETERS.L_Q6_EE,0,0));

    T_FL_Q5 = T_FL_Q4*T_Q4_Q5;
    T_FL_Q6 = T_FL_Q5*T_Q5_Q6;

    return T_FL_EE = T_FL_Q4*T_Q4_Q5*T_Q5_Q6*T_Q6_EE;
}


bool GeometricKinematic::calcInvKin(Eigen::Affine3d desEEPosition)
{
    //ROS_INFO_STREAM("desEEPosition: \n" << desEEPosition.matrix());
    //ROS_INFO_STREAM("RCM: \n" << RCM.matrix());
    Eigen::Affine3d T_0_Q6 = desEEPosition.translate(Eigen::Vector3d(-TOOL_PARAMETERS.L_Q6_EE,0,0));
    Eigen::Vector3d p_RCM_Q6 = T_0_Q6.translation()-RCM.translation();
    Eigen::Vector3d z_Q6 = T_0_Q6.matrix().col(2).head(3);
    Eigen::Vector3d nPlane = p_RCM_Q6.cross(z_Q6);

    Eigen::Vector3d y_Q6 = -T_0_Q6.matrix().col(1).head(3);

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
    //ROS_INFO_STREAM(jointAnglesTar(2));
    Eigen::Affine3d T_EE_Q5 = buildAffine3d(Eigen::Vector3d::Zero(),Eigen::Vector3d(90*DEG_TO_RAD,0,-jointAnglesTar(2)),true);
    T_EE_Q5.translate(Eigen::Vector3d(-TOOL_PARAMETERS.L_Q5_Q6,0,0));
    Eigen::Affine3d T_0_Q5 = desEEPosition*T_EE_Q5;
    //ROS_INFO_STREAM("T_0_Q5" << T_0_Q5.matrix());

    Eigen::Vector3d z_Q5 = T_0_Q5.matrix().col(2).head(3);

    Eigen::Vector3d y_Q5 = -T_0_Q5.matrix().col(1).head(3);

    Eigen::Vector3d p_Q5_RCM = T_0_Q5.translation() - RCM.translation();

    Eigen::Vector3d x_Q4 = p_Q5_RCM.cross(z_Q5);

    jointAnglesTar(1) = acos(x_Q4.dot(y_Q5)/(x_Q4.norm()*y_Q5.norm()));
    //ROS_INFO_STREAM("Q5:\n" << jointAnglesTar(1));

    if(isnan(jointAnglesTar(1)))
    {
        jointAnglesTar(1) = 0.0;
    }

    /*if(jointAnglesTar(1)>M_PI/2)
    {
        jointAnglesTar(1) = M_PI/2;
    }*/

    if(x_Q4.cross(y_Q5).dot(z_Q5)<0)
    {
        jointAnglesTar(1) = -jointAnglesTar(1);
    }

    Eigen::Affine3d T_Q5_Q4 = buildAffine3d(Eigen::Vector3d::Zero(),Eigen::Vector3d(-90*DEG_TO_RAD,0,-90*DEG_TO_RAD-jointAnglesTar(1)),true);
    T_0_Q4 = T_0_Q5*T_Q5_Q4;
    //ROS_INFO_STREAM("T_0_Q4: \n" << T_0_Q4.matrix());

    Eigen::Vector3d p_Q4_RCM = T_0_Q4.translation() - RCM.translation();

    Eigen::Vector3d y_FL = T_0_FL.matrix().col(1).head(3);

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

    Eigen::Affine3d T_Q4_FL = buildAffine3d(Eigen::Vector3d::Zero(),Eigen::Vector3d(-TOOL_PARAMETERS.A_0_Q4*DEG_TO_RAD,-TOOL_PARAMETERS.B_0_Q4*DEG_TO_RAD,-TOOL_PARAMETERS.C_0_Q4*DEG_TO_RAD-jointAnglesTar(0)),true);
    T_Q4_FL.translate(Eigen::Vector3d(-TOOL_PARAMETERS.X_0_Q4,-TOOL_PARAMETERS.Y_0_Q4,-TOOL_PARAMETERS.Z_0_Q4));
    T_0_FL = T_0_Q4*T_Q4_FL;
}

Eigen::Affine3d GeometricKinematic::buildAffine3d(const Eigen::Vector3d &translXYZ, const Eigen::Vector3d &axisZYX, bool zyx=true)
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

int main(int argc, char** argv)
{
    ros::init(argc,argv,"GeometricKinematicKinematics");
    ros::NodeHandle GeometricKinematicNH;
    GeometricKinematic* kinematic = new GeometricKinematic(GeometricKinematicNH);
}
