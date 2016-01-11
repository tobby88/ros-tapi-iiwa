#ifndef LAPAROSCOPE_H
#define LAPAROSCOPE_H

#include "kinematics.h"

class Laparoscope: public Kinematics
{
public:
    Laparoscope(const Eigen::Affine3d);
    void setAngles(const double, const double, const double);
    Eigen::Affine3d getT_FL_EE(){ return T_FL_EE;}
    Eigen::Affine3d getT_0_FL(){ return T_0_FL;}
    Eigen::Affine3d getT_0_Q4(){ return T_0_Q4;}
    double getQ4(){ return q4Tar;}
    double getQ5(){ return q5Tar;}
    double getQ6(){ return q6Tar;}

private:

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
        Eigen::Affine3d T_0_Q4;
        Eigen::Affine3d T_0_FL;
        double q4Tar;
        double q5Tar;
        double q6Tar;
        double q5Old;

    //geom Jacobi
    //Eigen::Affine3d T_0_Q4;
    Eigen::Affine3d T_0_Q5;
    Eigen::Affine3d T_0_Q6;



};

#endif // LAPAROSCOPE_H
