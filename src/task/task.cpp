#include "masterslave/task/task.h"



void Task::configurationCallback(masterslave::kinematicConfig &config, uint32_t level)
{
    apertureLimit = config.apertureLimit;
    gripperVelocityValue = config.gripperVelocity;
    heightSafety = config.safetyHeight;
}

Eigen::Quaternion<double> Task::QuaternionFromEuler(const Eigen::Vector3d &eulerXYZ, bool ZYX=true)
{
    Eigen::Quaternion<double> quat;
    quat.Identity();
    Eigen::AngleAxisd zAngle(eulerXYZ[2], Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yAngle(eulerXYZ[1], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd xAngle(eulerXYZ[0], Eigen::Vector3d::UnitX());
    if(ZYX)
        quat = zAngle * yAngle * xAngle;
    else
        quat = xAngle * yAngle * zAngle;

    return quat;
}




const double Task::DEG_TO_RAD = M_PI/180;
