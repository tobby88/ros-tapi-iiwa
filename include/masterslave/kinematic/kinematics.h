#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "Eigen/Dense"
#include "ros/ros.h"
#include "masterslave/descriptionparameters.h"

/*
 * Kinematics class as parent for different kinematics like URSUlA-Kinematic or Laparoscope-KINEMATIC
 *
 */

class Kinematics
{
    public:


    protected:

        // pure virtual methods for kinematic calculation
        virtual void calcInvKin(Eigen::Affine3d)=0;

        // help method to build up Eigen::Affine3d-Transformations
        Eigen::Affine3d buildAffine3d(const Eigen::Vector3d &translXYZ, const Eigen::Vector3d &axisZYX, bool zyx);

        //remote center of motion
        Eigen::Affine3d RCM;

        //Transformation from FL to Endeffector
        Eigen::Affine3d T_FL_EE;

        //Transformation from robot base to the endeffector
        Eigen::Affine3d T_0_EE;

        //Transformation from robot base to joint Q4
        Eigen::Affine3d T_0_Q4;

        Eigen::Affine3d T_FL_Q4;
        Eigen::Affine3d T_FL_Q5;
        Eigen::Affine3d T_FL_Q6;

        Eigen::VectorXd jointAnglesTar;
        Eigen::VectorXd jointAnglesAct;

        static constexpr double DEG_TO_RAD = M_PI/180;
        static constexpr double MM_TO_M = 1/1000;
};

#endif // KINEMATICS_H
