#ifndef GEOMETRICKINEMATIC_H
#define GEOMETRICKINEMATIC_H

#include "IKinematic.h"
#include "Eigen/Dense"
#include "ros/ros.h"
#include "masterslave/GeometricKinematicDirectKinematics.h"
#include "masterslave/GeometricKinematicInverseKinematics.h"
#include "masterslave/GeometricKinematicRCM.h"

#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>

class GeometricKinematic: public IKinematic
{
    public:
        GeometricKinematic(ros::NodeHandle&);
        Eigen::Affine3d getT_0_FL(){return T_0_FL;}

        Eigen::Affine3d getT_0_Q4(){return T_0_Q4;}

        void setAngles(const Eigen::VectorXd value);

    private:

        ros::NodeHandle nh_;


        static const toolDescriptionParameters TOOL_PARAMETERS;

        //Implementation of the inhereted methods of Kinematics class
        Eigen::Affine3d calcDirKin(Eigen::VectorXd);
        bool calcInvKin(Eigen::Affine3d);

        bool rcmCallback(masterslave::GeometricKinematicRCM::Request &req, masterslave::GeometricKinematicRCM::Response &resp);
        bool directKinematicsCallback(masterslave::GeometricKinematicDirectKinematics::Request &req, masterslave::GeometricKinematicDirectKinematics::Response &resp);
        bool inverseKinematicsCallback(masterslave::GeometricKinematicInverseKinematics::Request &req, masterslave::GeometricKinematicInverseKinematics::Response &resp);

        Eigen::Affine3d T_0_FL;
        ros::ServiceServer rcmServer;
        ros::ServiceServer directKinematicsServer;
        ros::ServiceServer inverseKinematicsServer;

        Eigen::Affine3d buildAffine3d(const Eigen::Vector3d &translXYZ, const Eigen::Vector3d &axisZYX, bool zyx);


};

#endif // GeometricKinematic_H
