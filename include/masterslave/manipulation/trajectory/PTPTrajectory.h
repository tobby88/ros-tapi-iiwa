#ifndef PTP_H
#define PTP_H

#include "Eigen/Dense"
#include "masterslave/TrajectoryGeneratorConfig.h"
#include "masterslave/manipulation/trajectory/ITrajectory.h"

class PTPTrajectory: public ITrajectory
{
    public:
        PTPTrajectory(Eigen::Affine3d startPoint, Eigen::Vector2i length, int zCoord, int speed, double cycleTime);
        Eigen::Affine3d calculateNextPoint();
    private:

};

#endif // PTP_H
