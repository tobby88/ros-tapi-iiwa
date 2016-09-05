#include "masterslave/manipulation/trajectory/LineTrajectory.h"

LineTrajectory::LineTrajectory(Eigen::Affine3d startPosition, Eigen::Vector3d RCM, Eigen::Vector3d firstPoint,
                               Eigen::Vector3d secondPoint, int speed, double cycleTime)
{
  speed_ = speed;
  cycleTime_ = cycleTime;

  startPosition_ = startPosition;
  currentPosition = startPosition;

  startPositionTrajectory_ = Eigen::Affine3d::Identity();
  startPositionTrajectory_.translate(firstPoint);
  startPositionTrajectory_.translate(RCM);

  endPositionTrajectory_.translate(secondPoint);
  endPositionTrajectory_.translate(RCM);

  trajectoryExecutionTime_ = (firstPoint - secondPoint).norm() / speed_;
  steps_ = floor(trajectoryExecutionTime_ / cycleTime_);

  coefficients[0] = startPositionTrajectory_.translation();
  coefficients[1] = Eigen::Vector3d::Zero();
  coefficients[2] = Eigen::Vector3d::Zero();

  coefficients[3] = 10 * (endPositionTrajectory_.translation() - startPositionTrajectory_.translation()) /
                    pow(trajectoryExecutionTime_, 3);
  coefficients[4] = -15 * (endPositionTrajectory_.translation() - startPositionTrajectory_.translation()) /
                    pow(trajectoryExecutionTime_, 4);
  coefficients[5] = 6 * (endPositionTrajectory_.translation() - startPositionTrajectory_.translation()) /
                    pow(trajectoryExecutionTime_, 5);
}

Eigen::Affine3d LineTrajectory::calculateNextPoint()
{
  if (pathParameterStart_ <= 1)
  {
    currentPosition.translate((startPositionTrajectory_.translation() - startPosition_.translation()) /
                              (startPositionTrajectory_.translation() - startPosition_.translation()).norm() * speed_ *
                              cycleTime_);
    pathParameterStart_ +=
        cycleTime_ / (startPositionTrajectory_.translation() - startPosition_.translation()).norm() * speed_;
    return currentPosition;
  }
  if (pathParameter_ < steps_)
  {
    for (const Eigen::Vector3d &coef : coefficients)
    {
      currentPosition.translation() += coef * cycleTime_;
    }
    pathParameter_++;
    return currentPosition;
  }
}
