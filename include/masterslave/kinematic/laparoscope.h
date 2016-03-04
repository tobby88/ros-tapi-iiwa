#ifndef LAPAROSCOPE_H
#define LAPAROSCOPE_H

#include "kinematics.h"
#include "Eigen/Dense"
#include "masterslave/descriptionparameters.h"
#include "ros/ros.h"
#include "masterslave/LaparoscopeDirectKinematics.h"
#include "masterslave/LaparoscopeInverseKinematics.h"
#include "masterslave/LaparoscopeRCM.h"

class Laparoscope: public Kinematics {
    public:
        Laparoscope(Eigen::Affine3d);
        Eigen::Affine3d getT_0_FL(){return T_0_FL;}

        Eigen::Affine3d getT_0_Q4(){return T_0_Q4;}

        void setAngles(const Eigen::VectorXd value);

    private:
        //Implementation of the inhereted methods of Kinematics class
        void calcDirKin();
        void calcInvKin();

        bool rcmCallback(masterslave::LaparoscopeRCM::request &req, masterslave::LaparoscopeRCM::response &resp);
        bool directKinematicsCallback(masterslave::LaparoscopeDirectKinematics::request &req, masterslave::LaparoscopeDirectKinematics::response &resp);
        bool inverseKinematicsCallback(masterslave::LaparoscopeInverseKinematics::request &req, masterslave::LaparoscopeInverseKinematics::response &resp);

        Eigen::Affine3d T_0_FL;
        ros::ServiceServer rcmServer;
        ros::ServiceServer directKinematicsServer;
        ros::ServiceServer inverseKinematicsServer;




};

#endif // LAPAROSCOPE_H
