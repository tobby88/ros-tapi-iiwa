#include "masterslave/manipulation/trajectory/PTPTrajectory.h"

PTPTrajectory::PTPTrajectory(Eigen::Affine3d startPoint, Eigen::Vector2i length, int zCoord, int speed, double cycleTime)
{
    cycleTime_ = cycleTime;
    ROS_INFO_STREAM("cycleTime: " << cycleTime_);
    startPositionTrajectory_ = startPoint;
    startPositionTrajectory_.translate(Eigen::Vector3d(-length(0)*0.5*M_TO_MM,-length(1)*0.5*M_TO_MM,zCoord*M_TO_MM-startPoint.translation().z()));
    endPositionTrajectory_ = startPoint;
    endPositionTrajectory_.translate(Eigen::Vector3d(length(0)*0.5*M_TO_MM, length(1)*0.5*M_TO_MM, zCoord*M_TO_MM-startPoint.translation().z()));
    speed_ = speed*M_TO_MM;
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
