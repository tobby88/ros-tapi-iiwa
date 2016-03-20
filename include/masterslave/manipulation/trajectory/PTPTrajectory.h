#ifndef PTP_H
#define PTP_H

#include "Eigen/Dense"
#include "masterslave/TrajectoryGeneratorConfig.h"
#include "masterslave/manipulation/trajectory/ITrajectory.h"

class PTPTrajectory: public ITrajectory
{
    public:
        PTPTrajectory(Eigen::Affine3d startPoint, Eigen::Vector3d RCM, Eigen::Vector3d firstPoint, Eigen::Vector3d secondPoint, int speed, double cycleTime);

        /**
         * @fn calculateNextPoint
         * @see ITrajectory
         */
        Eigen::Affine3d calculateNextPoint();
    private:


};

#endif // PTP_H
