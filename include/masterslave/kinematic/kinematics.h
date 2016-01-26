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
        //Transform of the joint Q4
        Eigen::Affine3d geT_0_Q4(){return T_0_Q4;}

        //Remote Center of Motion
        Eigen::Affine3d getRCM(){return RCM;}

        //setter and calculation of the RCM
        void setRCM(Eigen::Affine3d);

        //setter for the endeffector position of the tool
        void setT_0_EE(Eigen::Affine3d);

        //inline definition of the getter
        Eigen::Affine3d getT_FL_EE(){return T_FL_EE;}
        toolAngles getAngles(){ return toolAnglesTar;}


    protected:

        // pure virtual methods for kinematic calculation
        virtual void calcDirKin()=0;
        virtual void calcInvKin()=0;

        //method to calculate the direct kinematic of the Laparoscope
        Eigen::Affine3d calcLaparoscopeDirKin();

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

        toolAngles toolAnglesTar;
        toolAngles toolAnglesAct;

        static const toolDescriptionParameters TOOL_PARAMETERS;
};

#endif // KINEMATICS_H
