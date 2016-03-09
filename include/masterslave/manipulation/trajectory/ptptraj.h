#ifndef PTP_H
#define PTP_H

#include "Eigen/Dense"
#include "masterslave/trajectorygeneratorConfig.h"
#include "masterslave/manipulation/trajectory/trajectory.h"

class PTPTraj: public Trajectory
{
    public:
        PTPTraj(Eigen::Affine3d startPoint, Eigen::Vector2i length, int zCoord, int speed, double cycleTime);
        Eigen::Affine3d calculateNextPoint();
    private:

};

#endif // PTP_H
