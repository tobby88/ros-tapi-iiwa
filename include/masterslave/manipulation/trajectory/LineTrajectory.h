#ifndef LINETRAJECTORY_H
#define LINETRAJECTORY_H

#include "ITrajectory.h"
#include <array>

/**
 * @file LineTrajectory.h
 *
 * @class LineTrajectory
 * @brief Eine Klasse, die eine Interpolation eines Polynom fünften Grades zwischen zwei Punkten ermöglicht
 *
 * @author Fabian Baier
 * @date 20.03.2016
 */

class LineTrajectory : public ITrajectory
{
    public:
        LineTrajectory(Eigen::Affine3d startPosition, Eigen::Vector3d RCM,Eigen::Vector3d firstPoint, Eigen::Vector3d secondPoint, int speed, double cycleTime);

        /**
         * @fn calculateNextPoint
         * @see ITrajectory
         */
        Eigen::Affine3d calculateNextPoint();
    private:

        /**
         * @var coefficients
         * @brief Koeffizienten des Polynoms fünften Grades
         */
        std::array<Eigen::Vector3d,6> coefficients;

        /**
         * @var trajectoryExecutionTime_
         * @brief Zeit für die Interpolation zwischen den zwei Punkten in Abhängigkeit der gewünschten durchschnittlichen Geschwindigkeit
         */
        double trajectoryExecutionTime_;

        /**
         * @var steps_
         * @brief Anzahl der Interpolationsschritte in Abhängigkeit der Länge der Trajektore, der Bahngeschwindigkeit und der Zykluszeit
         */
        int steps_;

        /**
         * @var cycleTime_
         * @brief Zykluszeit
         */
        double cycleTime_;

};

#endif // LINETRAJECTORY_H
