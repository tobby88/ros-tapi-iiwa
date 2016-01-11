#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Eigen/Geometry>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "ros/ros.h"

// geometric tool description
struct toolDescriptionParameters{
    const double X_0_Q4;
    const double Y_0_Q4;
    const double Z_0_Q4;
    const double A_0_Q4;
    const double B_0_Q4;
    const double C_0_Q4;
    const double L_Q5_Q6;
    const double L_Q6_EE;
    const double X_RCM;
};


class Kinematics
{
public:
    void setT_0_EE(Eigen::Affine3d);
    Eigen::Affine3d getRCM(){ return RCM;}

protected:
    //help functions
    Eigen::Affine3d buildAffine3d(const Eigen::Vector3d &translXYZ, const Eigen::Vector3d &axisZYX, bool zyx);
    void buildDebugFrameFromTM(const Eigen::Affine3d &T_0_XX, const std::string &name);

    virtual void calcDirKin()=0;
    virtual void calcInvKin()=0;

    //Remote Center of Motion
    Eigen::Affine3d RCM;
    //Inv Kin
    Eigen::Affine3d T_0_EE;

    static const toolDescriptionParameters toolParameters;







};

#endif // KINEMATICS_H
