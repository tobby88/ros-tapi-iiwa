#ifndef IKINEMATIC_H
#define IKINEMATIC_H

#include "Eigen/Dense"
#include "ros/ros.h"
#include "masterslave/DescriptionParameters.h"

#include <dynamic_reconfigure/server.h>


/**
 * @file IKinematic.h
 *
 * @class IKinematic
 * @brief Abstrakte Basisklasse für die Kinematikberechnung
 *
 * @author Fabian Baier
 * @date 20.03.2016
 */

class IKinematic
{
    public:


    protected:
        /**
         * @fn checkTCP
         * @brief Arbeitsraumüberwachung
         * @deprecated wird in der neuen Version entfernt, da es eigene Überwachungsklassen gibt
         * @param TCP Aktueller TCP
         * @return Flag, ob der definierte Arbeitsraum eingehalten wurde
         * @see BoundingBox
         */
        bool checkTCP(Eigen::Affine3d TCP);

        /**
         * @fn calcInvKin
         * @brief Abstrakte Methode zur Berechnung der inversen Kinematik
         * @param T_0_EE Aktuelle TCP-Lage
         * @return Arbeitsraumgrenzen eingehalten?
         */
        virtual bool calcInvKin(Eigen::Affine3d T_0_EE)=0;

        /**
         * @fn buildAffine3d
         * @brief Hilfsfunktion zur Erstellung von Affinen Transformationen aus einem Translationsvekt und den RPY-Winkeln
         * @param translXYZ Translationsvektor
         * @param axisZYX Rotationsvektor
         * @param zyx Flag zur Festlegung der Rotationsrichtung
         * @return Affine Transformation
         */
        Eigen::Affine3d buildAffine3d(const Eigen::Vector3d &translXYZ, const Eigen::Vector3d &axisZYX, bool zyx);

        /**
         * @var RCM
         * @brief Trokarpunkt im Weltkoordinatensystem
         */
        Eigen::Affine3d RCM;

        /**
         * @var T_FL_EE
         * @brief Transformation zwischen Endeffektor und Roboterflansch
         */
        Eigen::Affine3d T_FL_EE;

        /**
         * @var T_0_EE
         * @brief Endeffektorlage im Weltkoordinatensystem
         */
        Eigen::Affine3d T_0_EE;

        /**
         * @var T_0_Q4
         * @brief Lage des ersten Werkzeuggelenks im Weltkoordinatensystem
         * @todo Eindeutige Namenskonvention Q4tool und Q4lbr
         */

        Eigen::Affine3d T_0_Q4;

        /**
         * @var T_FL_Q4
         * @brief Lage des ersten Werkzeuggelenks im Roboterflanschkoordinatensystem
         */
        Eigen::Affine3d T_FL_Q4;

        /**
         * @var T_FL_Q5
         * @brief Lage des zweiten Werkzeuggelenks im Roboterflanschkoordinatensystem
         */
        Eigen::Affine3d T_FL_Q5;

        /**
         * @var T_FL_Q6
         * @brief Lage des dritten Werkzeuggelenks im Roboterflanschkoordinatensystem
         */
        Eigen::Affine3d T_FL_Q6;

        /**
         * @var jointAnglesTar
         * @brief Vektor der gewünschten Gelenkwinkel des Werkzeuges bzw. des Gesamtsystems
         */
        Eigen::VectorXd jointAnglesTar;

        /**
         * @var jointAnglesAct
         * @brief Vektor der aktuellen Gelenkwinkel des Werkzeuges bzw. des Gesamtsystems
         */
        Eigen::VectorXd jointAnglesAct;

        const double DEG_TO_RAD{M_PI/180};
        const double MM_TO_M{1/1000};
        /**
         * @var apertureMax
         * @deprecated
         */
        int apertureMax{60};

        /**
         * @var penetrationMax
         * @deprecated
         */
        double penetrationMax{0.3};

        /**
         * @var penetrationMin
         * @deprecated
         */
        double penetrationMin{0.1};

        /**
         * @var rcmServiceCalled
         * @brief Flag, ob der Trokarpunkt berechnet ist
         */
        bool rcmServiceCalled{false};

        /**
         * @var directKinematicsServiceCalled
         * @brief Flag, ob der aktuelle TCP bekannt ist
         */
        bool directKinematicsServiceCalled{false};

        /**
         * @var rcmServiceServer
         * @brief ServiceServer für die Bestimmung des Trokarpunktes
         */
        ros::ServiceServer rcmServiceServer;

        /**
         * @var directKinematicsServer
         * @brief ServiceServer für die Berechnung des TCP
         */
        ros::ServiceServer directKinematicsServer;

        /**
         * @var inverseKinematicsServer
         * @brief ServiceServer für die Berechnung der Gelenkwinkel und der Roboterflanschlage im Weltkoordinatensystem
         */
        ros::ServiceServer inverseKinematicsServer;

};

#endif // KINEMATICS_H
