#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "Eigen/Dense"
#include "ros/ros.h"
#include "masterslave/descriptionparameters.h"

class Kinematics
{
    public:
        Eigen::Affine3d geT_0_Q4(){return T_0_Q4;}
        Eigen::Affine3d getRCM(){return RCM;}
        void setRCM(Eigen::Affine3d);
        void setT_0_EE(Eigen::Affine3d);
        Eigen::Affine3d getT_0_Q4(){return T_0_Q4;}
        Eigen::Affine3d getT_FL_EE(){return T_FL_EE;}
        toolAngles getAngles(){ return toolAnglesTar;}


    protected:
        virtual void calcDirKin()=0;
        virtual void calcInvKin()=0;
        Eigen::Affine3d calcLaparoscopeDirKin();
        Eigen::Affine3d buildAffine3d(const Eigen::Vector3d &translXYZ, const Eigen::Vector3d &axisZYX, bool zyx);

        Eigen::Affine3d RCM;
        Eigen::Affine3d T_FL_EE;
        Eigen::Affine3d T_0_EE;

        Eigen::Affine3d T_0_Q4;

        toolAngles toolAnglesTar;
        toolAngles toolAnglesAct;

        static const toolDescriptionParameters TOOL_PARAMETERS;
};

#endif // KINEMATICS_H
