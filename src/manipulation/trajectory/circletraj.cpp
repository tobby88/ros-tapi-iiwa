#include "masterslave/manipulation/trajectory/circletraj.h"

CircleTraj::CircleTraj(Eigen::Affine3d startPoint, int radius, int zCoord, int speed , double cycleTime)
{
    radius_ = radius*M_TO_MM;
    cycleTime_ = cycleTime;
    startPosition_ = startPoint;
    startPositionTrajectory_ = startPoint;
    startPositionTrajectory_.translate(Eigen::Vector3d(radius_,0,zCoord*M_TO_MM-startPoint.translation().z()));
    circleCenter = startPoint;
    circleCenter.translate(Eigen::Vector3d(0,0,zCoord*M_TO_MM-startPoint.translation().z()));
    currentPosition = startPoint;
    pathParameterStart_=0;
    pathParameter_=0;
    speed_ = speed*M_TO_MM;
}

Eigen::Affine3d CircleTraj::calculateNextPoint()
{

    if(pathParameterStart_<=1)
    {
        currentPosition.translate((startPositionTrajectory_.translation()-startPosition_.translation())/Eigen::Vector3d(startPositionTrajectory_.translation()-startPosition_.translation()).norm()*cycleTime_*speed_);
        pathParameterStart_ += cycleTime_/Eigen::Vector3d(startPositionTrajectory_.translation()-startPosition_.translation()).norm()*speed_;
    }
    else
    {

        Eigen::Vector3d incrementVector = Eigen::Vector3d(radius_*cos(2*M_PI*pathParameter_),radius_*sin(2*M_PI*pathParameter_),0);
        currentPosition.translation() = circleCenter.translation() + incrementVector;
        pathIncrement = cycleTime_/(radius_*2*M_PI)*speed_;
        pathParameter_ += pathIncrement;
    }
    return currentPosition;
}
