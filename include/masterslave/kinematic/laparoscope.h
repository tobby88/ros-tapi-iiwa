#ifndef LAPAROSCOPE_H
#define LAPAROSCOPE_H

#include "kinematics.h"
#include "Eigen/Dense"
#include "masterslave/descriptionparameters.h"
#include "ros/ros.h"
#include "masterslave/LaparoscopeDirectKinematics.h"
#include "masterslave/LaparoscopeInverseKinematics.h"
#include "masterslave/LaparoscopeRCM.h"

#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>

class Laparoscope: public Kinematics
{
    public:
        Laparoscope(ros::NodeHandle&);
        Eigen::Affine3d getT_0_FL(){return T_0_FL;}

        Eigen::Affine3d getT_0_Q4(){return T_0_Q4;}

        void setAngles(const Eigen::VectorXd value);

    private:

        ros::NodeHandle nh_;


        static const toolDescriptionParameters TOOL_PARAMETERS;

        //Implementation of the inhereted methods of Kinematics class
        Eigen::Affine3d calcDirKin(Eigen::VectorXd);
        void calcInvKin(Eigen::Affine3d);

        bool rcmCallback(masterslave::LaparoscopeRCM::Request &req, masterslave::LaparoscopeRCM::Response &resp);
        bool directKinematicsCallback(masterslave::LaparoscopeDirectKinematics::Request &req, masterslave::LaparoscopeDirectKinematics::Response &resp);
        bool inverseKinematicsCallback(masterslave::LaparoscopeInverseKinematics::Request &req, masterslave::LaparoscopeInverseKinematics::Response &resp);

        Eigen::Affine3d T_0_FL;
        ros::ServiceServer rcmServer;
        ros::ServiceServer directKinematicsServer;
        ros::ServiceServer inverseKinematicsServer;

        Eigen::Affine3d buildAffine3d(const Eigen::Vector3d &translXYZ, const Eigen::Vector3d &axisZYX, bool zyx);


};

#endif // LAPAROSCOPE_H
