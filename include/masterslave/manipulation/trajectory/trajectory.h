#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "Eigen/Core"
#include "Eigen/Dense"

/*
 * Abstract Base Class for all Trajectory types
 *
 */

class Trajectory
{
public:
    virtual Eigen::Affine3d calculateNextPoint()=0;
    void setSpeed(double speed){ speed_ = speed*M_TO_MM;}
protected:
    Eigen::Affine3d startPositionTrajectory_;
    Eigen::Affine3d endPositionTrajectory_;
    Eigen::Affine3d startPosition_;
    Eigen::Vector3d translation;
    Eigen::Affine3d currentPosition;
    double pathParameterStart_;
    double pathIncrement;
    double cycleTime_;
    double speed_;
    double pathParameter_;

    const double M_TO_MM{0.001};

};

#endif // TRAJECTORY_H
