#ifndef ITRAJECTORY_H
#define ITRAJECTORY_H

#include "Eigen/Core"
#include "Eigen/Dense"

/*
 * Abstract Base Class for all Trajectory types
 *
 */

class ITrajectory
{
public:
    virtual Eigen::Affine3d calculateNextPoint()=0;
    void setSpeed(double speed){ speed_ = speed;}
protected:
    Eigen::Affine3d startPositionTrajectory_;
    Eigen::Affine3d endPositionTrajectory_;
    Eigen::Affine3d startPosition_;
    Eigen::Affine3d currentPosition;
    double pathParameterStart_{0};
    double pathIncrement;
    double cycleTime_;
    double speed_;
    double pathParameter_{0};

};

#endif // TRAJECTORY_H
