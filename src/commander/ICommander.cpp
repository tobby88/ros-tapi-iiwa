#include "masterslave/commander/ICommander.h"



void ICommander::configurationCallback(masterslave::MasterSlaveConfig &config, uint32_t level)
{
    state = static_cast<OPENIGTL_STATE>(config.cur_state);
    rosRate = config.rosRate;
    ROS_INFO("STATE CHANGED");
}

Eigen::Quaternion<double> ICommander::QuaternionFromEuler(const Eigen::Vector3d &eulerXYZ, bool ZYX=true)
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



