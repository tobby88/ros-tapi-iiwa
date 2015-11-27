#ifndef LAPAROSCOPE_H
#define LAPAROSCOPE_H

#include <Eigen/Geometry>
#include <Eigen/Core>
//#include <Eigen/Eigen>
#include "ros/ros.h"

class Laparoscope
{
public:
    Laparoscope(const Eigen::Affine3d);
    void setAngles(const double, const double, const double);
    void setT_0_EE(Eigen::Affine3d);
    Eigen::Affine3d getT_FL_EE(){ return T_FL_EE;}
    Eigen::Affine3d getT_0_FL(){ return T_0_FL;}
    double getQ4(){ return q4Tar;}
    double getQ5(){ return q5Tar;}
    double getQ6(){ return q6Tar;}

private:
    //help functions
    Eigen::Affine3d buildAffine3d(const Eigen::Vector3d &translXYZ, const Eigen::Vector3d &axisZYX, bool zyx);
    void buildDebugFrameFromTM(const Eigen::Affine3d &T_0_XX, const std::string &name);
    //kinematic calculations
    void calcDirKin();
    void calcInvKin();
    //Remote Center of Motion
    Eigen::Affine3d RCM;
    //dirKin
    double q4Act;
    double q5Act;
    double q6Act;
    Eigen::Affine3d T_FL_EE;
    //invKin
    Eigen::Affine3d T_0_EE;
    Eigen::Affine3d T_0_FL;
    double q4Tar;
    double q5Tar;
    double q6Tar;
    double q5Old;
    // geometric tool description
    struct toolDescriptionParameters{
        double X_0_Q4;
        double Y_0_Q4;
        double Z_0_Q4;
        double A_0_Q4;
        double B_0_Q4;
        double C_0_Q4;
        double L_Q5_Q6;
        double L_Q6_EE;
    } toolParameters;



};

#endif // LAPAROSCOPE_H
