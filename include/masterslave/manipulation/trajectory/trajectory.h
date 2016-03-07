#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "Eigen/Core"


class Trajectory
{
public:
    virtual Eigen::Affine3d calculateNextPoint()=0;
protected:

    Eigen::Affine3d startPosition_;
    Eigen::Affine3d endPosition_;
    double cycleTime_;
    double speed_;

};

#endif // TRAJECTORY_H
