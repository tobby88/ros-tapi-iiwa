#ifndef ITRAJECTORY_H
#define ITRAJECTORY_H

#include "Eigen/Core"
#include "Eigen/Dense"
#include <iostream>
#include "ros/ros.h"
/**
 * @file ITrajectory.h
 *
 * @class ITrajectory
 * @brief Abstrakte Basisklasse für verschiedene Trajektorienarten
 *
 * @author Fabian Baier
 * @date 20.03.2016
 */

class ITrajectory
{
public:

    /**
     * @fn calculateNextPoint
     * @brief Abstrakte Funktion zur Berechnung des nächsten Bahnpunktes
     * @return Nächster Bahnpunkt
     */
    virtual Eigen::Affine3d calculateNextPoint()=0;

    /**
     * @fn setSpeed
     * @brief Zugriff auf die Bahngeschwindigkeit
     * @param speed Geschwindigkeit in [mm/s]
     */
    void setSpeed(double speed)
    {
        speed_ = speed;
        ROS_INFO_STREAM(speed_);
    }
protected:

    /**
     * @var startPositionTrajectory_
     * @brief Startpunkt der gewünschten Trajektorie
     * @see calculateNextPoint
     */
    Eigen::Affine3d startPositionTrajectory_;

    /**
     * @var endPositionTrajectory_
     * @brief Endpunkt der gewünschten Trajektorie
     * @see calculateNextPoint
     */
    Eigen::Affine3d endPositionTrajectory_;

    /**
     * @var startPosition_
     * @brief Aktuelle TCP-Lage und Startpunkt der PTP-Bewegung zu startPositionTrajectory_
     * @see calculateNextPoint
     */
    Eigen::Affine3d startPosition_;

    /**
     * @var currentPosition
     * @brief Aktuelle TCP-Lage während der Bahninterpolation
     * @see calculateNextPoint
     */
    Eigen::Affine3d currentPosition;

    /**
     * @var pathParameterStart_
     * @brief Pfadvariable der PTP-Bewegung zwischen startPosition_ und startPositionTrajectory_
     */
    double pathParameterStart_{0};

    /**
     * @var pathIncrement
     * @brief Der inkrementelle Teil der Bahn zwischen zwei Bahnpunkten
     */
    double pathIncrement;

    /**
     * @var cycleTime_
     * @brief Zykluszeit
     */
    double cycleTime_;

    /**
     * @var speed_
     * @brief Bahngeschwindigkeit in [mm/s]
     */
    double speed_;

    /**
     * @var pathParameter_
     * @brief Pfadvariable der jeweiligen Trajektorie
     */
    double pathParameter_{0};

};

#endif // TRAJECTORY_H
