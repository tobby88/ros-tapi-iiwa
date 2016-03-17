#include "masterslave/kinematic/IKinematic.h"

#define DEG_TO_RAD M_PI/180

bool IKinematic::checkTCP(Eigen::Affine3d TCP)
{
    Eigen::Vector3d xEE_RCM = TCP.translation() - RCM.translation();
    ROS_INFO_STREAM("xEE_RCM: \n" << xEE_RCM << "\n RCM: \n" << RCM.translation());
    //Öffnungswinkel des Kegels
    double aperture = asin(xEE_RCM.head(2).norm()/xEE_RCM[2]);
    double polarAngle = atan2(xEE_RCM[1],xEE_RCM[0]);
    if(std::abs(xEE_RCM.norm()) > penetrationMax || std::abs(xEE_RCM.norm()) < penetrationMin || apertureMax*DEG_TO_RAD < aperture)
    {
        return false;
    }
    return true;
}


