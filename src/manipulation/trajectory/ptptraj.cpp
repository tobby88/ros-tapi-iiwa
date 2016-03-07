#include "masterslave/manipulation/trajectory/ptptraj.h"

PTPTraj::PTPTraj(Eigen::Affine3d startPoint, masterslave::trajectorygeneratorConfig& config, double cycleTime)
{
    cycleTime_ = cycleTime;
    startPosition_ = startPoint.translate(Eigen::Vector3d(-config.LengthX/2,-config.LengthY/2,0));
    endPosition_ = startPoint.translate(Eigen::Vector3d(config.LengthX/2, config.LengthY/2, 0));
    translation = Eigen::Vector3d(config.LengthX,config.LengthY,0);
    speed_ = config.Speed;
    currentPosition = startPosition_;
}

Eigen::Affine3d PTPTraj::calculateNextPoint()
{
    currentPosition.translate(translation*cycleTime_/speed_);
    return currentPosition;
}
