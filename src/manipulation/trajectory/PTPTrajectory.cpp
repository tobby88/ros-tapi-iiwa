#include "masterslave/manipulation/trajectory/PTPTrajectory.h"

PTPTrajectory::PTPTrajectory(Eigen::Affine3d startPoint, Eigen::Vector3d RCM, Eigen::Vector3d firstPoint, Eigen::Vector3d secondPoint, int speed, double cycleTime)
{
    cycleTime_ = cycleTime;

    startPositionTrajectory_ = Eigen::Affine3d::Identity();
    startPositionTrajectory_.translate(RCM);
    startPositionTrajectory_.translate(firstPoint);
    endPositionTrajectory_ = Eigen::Affine3d::Identity();
    endPositionTrajectory_.translate(RCM);
    endPositionTrajectory_.translate(secondPoint);

    speed_ = speed;
    currentPosition = startPoint;
    startPosition_ = startPoint;
}

Eigen::Affine3d PTPTrajectory::calculateNextPoint()
{
    // Movement from the current position to the start position of the trajectory
    if(pathParameterStart_<=1)
    {
        currentPosition.translate((startPositionTrajectory_.translation()-startPosition_.translation())/(startPositionTrajectory_.translation()-startPosition_.translation()).norm()*speed_*cycleTime_);
        pathParameterStart_ += cycleTime_/(startPositionTrajectory_.translation()-startPosition_.translation()).norm()*speed_;
        return currentPosition;
    }
    if(pathParameter_<=0)
    {
        pathIncrement = cycleTime_/(startPositionTrajectory_.translation()-startPosition_.translation()).norm()*speed_;
    }
    if(pathParameter_>=1)
    {
        pathIncrement = -cycleTime_/(startPositionTrajectory_.translation()-startPosition_.translation()).norm()*speed_;
    }
    currentPosition.translate((endPositionTrajectory_.translation()-startPositionTrajectory_.translation())*pathIncrement);
    pathParameter_ += pathIncrement;
    return currentPosition;
}
