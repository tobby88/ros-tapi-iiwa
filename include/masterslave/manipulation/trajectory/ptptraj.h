#ifndef PTP_H
#define PTP_H

#include "Eigen/Dense"
#include "masterslave/trajectorygeneratorConfig.h"
#include "masterslave/manipulation/trajectory/trajectory.h"

class PTPTraj: public Trajectory
{
    public:
        PTPTraj(Eigen::Affine3d startPoint, masterslave::trajectorygeneratorConfig&, double cycleTime);
        Eigen::Affine3d calculateNextPoint();
    private:
        Eigen::Vector3d translation;
        Eigen::Affine3d currentPosition;


};

#endif // PTP_H
