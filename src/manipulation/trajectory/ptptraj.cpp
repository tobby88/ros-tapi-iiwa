#include "masterslave/manipulation/trajectory/ptptraj.h"

PTPTraj::PTPTraj(Eigen::Affine3d startPoint, Eigen::Vector2i length, int zCoord, int speed, double cycleTime)
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
    pathParameterStart_=0;
    pathParameter_=0;
}

Eigen::Affine3d PTPTraj::calculateNextPoint()
{
    if(pathParameterStart_<=1)
    {
        currentPosition.translate((startPositionTrajectory_.translation()-startPosition_.translation())/(startPositionTrajectory_.translation()-startPosition_.translation()).norm()*speed_*cycleTime_);
        pathParameterStart_ += cycleTime_/(startPositionTrajectory_.translation()-startPosition_.translation()).norm()*speed_;
        ROS_INFO_STREAM("pathParameter: " << pathParameterStart_);
    }
    else
    {

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
        ROS_INFO_STREAM("pathParameter: " << pathParameter_);

    }
    return currentPosition;
}
