#include "masterslave/manipulation/trajectory/CircleTrajectory.h"

CircleTrajectory::CircleTrajectory(Eigen::Affine3d startPoint, int radius, int zCoord, int speed , double cycleTime)
{
    radius_ = radius*M_TO_MM;
    cycleTime_ = cycleTime;
    startPosition_ = startPoint;
    startPositionTrajectory_ = startPoint;
    startPositionTrajectory_.translate(Eigen::Vector3d(radius_,0,zCoord*M_TO_MM-startPoint.translation().z()));
    circleCenter = startPoint;
    circleCenter.translate(Eigen::Vector3d(0,0,zCoord*M_TO_MM-startPoint.translation().z()));
    currentPosition = startPoint;
    speed_ = speed*M_TO_MM;
}

Eigen::Affine3d CircleTrajectory::calculateNextPoint()
{
    // Start movement to the start position from the current position
    if(pathParameterStart_<=1)
    {
        currentPosition.translate((startPositionTrajectory_.translation()-startPosition_.translation())/Eigen::Vector3d(startPositionTrajectory_.translation()-startPosition_.translation()).norm()*cycleTime_*speed_);
        // 0 <= Path parameter <= 1
        pathParameterStart_ += cycleTime_/Eigen::Vector3d(startPositionTrajectory_.translation()-startPosition_.translation()).norm()*speed_;
        return currentPosition;
    }
    // current part of the translation
    Eigen::Vector3d incrementVector = Eigen::Vector3d(radius_*cos(2*M_PI*pathParameter_),radius_*sin(2*M_PI*pathParameter_),0);
    // translate around the circle center
    currentPosition.translation() = circleCenter.translation() + incrementVector;
    // path increment to control the speed out of rqt
    pathIncrement = cycleTime_/(radius_*2*M_PI)*speed_;
    pathParameter_ += pathIncrement;
    if(pathParameter_>=1) pathParameter_-=1;
    return currentPosition;
}
