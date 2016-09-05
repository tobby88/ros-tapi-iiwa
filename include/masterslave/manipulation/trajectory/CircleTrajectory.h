#ifndef CIRCLETRAJECTORY_H
#define CIRCLETRAJECTORY_H

#include "ITrajectory.h"

#include "Eigen/Core"
#include "Eigen/Dense"

/**
 * @file CircleTrajectory.h
 *
 * @class CircleTrajectory
 * @brief Klasse, die eine Interpolation einer Kreistrajektorie ermöglicht
 *
 * @author Fabian Baier
 * @date 20.03.2016
 */

class CircleTrajectory : public ITrajectory
{
public:
  CircleTrajectory(Eigen::Affine3d startPoint, Eigen::Vector3d RCM, double radius, double zCoord, double speed,
                   double cycleTime);

  /**
   * @fn calculateNextPoint
   * @see ITrajectory
   */
  Eigen::Affine3d calculateNextPoint();

private:
  /**
   * @var circleCenter
   * @brief Zentroid der Kreistrajektorie
   */
  Eigen::Affine3d circleCenter;

  /**
   * @var radius_
   * @brief Radius der Kreistrajektorie
   */
  double radius_;
};

#endif  // CIRCLE_H
