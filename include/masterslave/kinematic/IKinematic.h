#ifndef IKINEMATIC_H
#define IKINEMATIC_H

#include "Eigen/Dense"
#include "ros/ros.h"
#include "masterslave/DescriptionParameters.h"

#include <dynamic_reconfigure/server.h>


/*
 * Kinematics class as parent for different kinematics like URSUlA-Kinematic or Laparoscope-KINEMATIC
 *
 */

class IKinematic
{
    public:


    protected:

        bool checkTCP(Eigen::Affine3d TCP);

        // pure virtual methods for kinematic calculation
        virtual bool calcInvKin(Eigen::Affine3d)=0;

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

        const double DEG_TO_RAD{M_PI/180};
        const double MM_TO_M{1/1000};

        int apertureMax{60};
        double penetrationMax{0.3};
        double penetrationMin{0.1};




        bool rcmServiceCalled{false};
        bool directKinematicsServiceCalled{false};
};

#endif // KINEMATICS_H
