#include "masterslave/manipulation/trajectory/CircleTrajectory.h"

#include "ros/ros.h"

CircleTrajectory::CircleTrajectory(Eigen::Affine3d startPoint, Eigen::Vector3d RCM, double radius, double zCoord, double speed , double cycleTime)
{
    ROS_INFO_STREAM("RCM" << RCM);
    radius_ = radius;
    cycleTime_ = cycleTime;
    currentPosition = startPoint;
    speed_ = speed;
    ROS_INFO_STREAM("speed: " << speed_);
    startPosition_ = startPoint;
    startPositionTrajectory_ = Eigen::Affine3d::Identity();
    startPositionTrajectory_.translate(RCM);
    startPositionTrajectory_.translate(Eigen::Vector3d(radius_,0,zCoord-startPoint.translation().z()));
    circleCenter = Eigen::Affine3d::Identity();
    circleCenter.translate(RCM);
    circleCenter.translate(Eigen::Vector3d(0,0,zCoord-startPoint.translation().z()));
}

Eigen::Affine3d CircleTrajectory::calculateNextPoint()
{
    // Start movement to the start position from the current position
    if(pathParameterStart_<=1)
    {
        currentPosition.translation() += ((startPositionTrajectory_.translation()-startPosition_.translation())/Eigen::Vector3d(startPositionTrajectory_.translation()-startPosition_.translation()).norm()*cycleTime_*speed_);
        // 0 <= Path parameter <= 1
        pathParameterStart_ += cycleTime_/Eigen::Vector3d(startPositionTrajectory_.translation()-startPosition_.translation()).norm()*speed_;
        //ROS_WARN_STREAM(pathParameterStart_);
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
    //ROS_WARN_STREAM(currentPosition.translation());
    return currentPosition;
}
