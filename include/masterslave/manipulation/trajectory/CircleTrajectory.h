#ifndef CIRCLETRAJECTORY_H
#define CIRCLETRAJECTORY_H

#include "ITrajectory.h"



#include "Eigen/Core"
#include "Eigen/Dense"

class CircleTrajectory: public ITrajectory
{
    public:
        CircleTrajectory(Eigen::Affine3d startPoint, Eigen::Vector3d RCM, double radius, double zCoord, double speed , double cycleTime);
        Eigen::Affine3d calculateNextPoint();
    private:
        Eigen::Affine3d circleCenter;
        double radius_;
};

#endif // CIRCLE_H
