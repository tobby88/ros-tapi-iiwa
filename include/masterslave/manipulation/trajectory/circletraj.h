#ifndef CIRCLE_H
#define CIRCLE_H

#include "masterslave/manipulation/trajectory/trajectory.h"



#include "Eigen/Core"
#include "Eigen/Dense"

class CircleTraj: public Trajectory
{
    public:
        CircleTraj(Eigen::Affine3d startPoint, int radius, int zCoord, int speed , double cycleTime);
        Eigen::Affine3d calculateNextPoint();
    private:
        Eigen::Affine3d circleCenter;
        double radius_;
};

#endif // CIRCLE_H
