#ifndef LINETRAJECTORY_H
#define LINETRAJECTORY_H

#include "ITrajectory.h"
#include <array>

class LineTrajectory : public ITrajectory
{
    public:
        LineTrajectory(Eigen::Affine3d startPosition, Eigen::Vector3d RCM,Eigen::Vector3d firstPoint, Eigen::Vector3d secondPoint, int speed, double cycleTime);
        Eigen::Affine3d calculateNextPoint();
    private:
        std::array<Eigen::Vector3d,6> coefficients;
        double trajectoryExecutionTime_;
        int steps_;
        double cycleTime_;

};

#endif // LINETRAJECTORY_H
