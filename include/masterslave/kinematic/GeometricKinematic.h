#ifndef GEOMETRICKINEMATIC_H
#define GEOMETRICKINEMATIC_H

#include "IKinematic.h"
#include "Eigen/Dense"
#include "ros/ros.h"
#include "masterslave/GeometricKinematicDirectKinematics.h"
#include "masterslave/GeometricKinematicInverseKinematics.h"
#include "masterslave/GeometricKinematicRCM.h"

#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>

/**
 * @file GeometricKinematic.h
 *
 * @class GeometricKinematic
 * @brief Eine Klasse zur Berechnung der Kinematik des Werkzeuges und der Endeffektorlage des LBR iiwa
 *
 * @author Fabian Baier
 * @date 20.03.2016
 * @see IKinematic
 */

class GeometricKinematic: public IKinematic
{
    public:
        GeometricKinematic(ros::NodeHandle&);

        /**
         * @fn getT_0_FL
         * @todo check ob noch benötigt
         * @return
         */
        Eigen::Affine3d getT_0_FL(){return T_0_FL;}

        /**
         * @fn getT_0_Q4
         * @todo check ob noch benötigt
         * @return
         */
        Eigen::Affine3d getT_0_Q4(){return T_0_Q4;}

        /**
         * @fn setAngles
         * @param value
         * @todo check ob noch benötigt, da jetzt über Services kommuniziert wird
         */
        void setAngles(const Eigen::VectorXd value);

    private:

        ros::NodeHandle nh_;

        /**
         * @var TOOL_PARAMETERS
         * @brief Konstante, die die geometrischen Parameters des laparoskopischen Werkzeuges festlegen
         * (X_FL_Q4, Y_FL_Q4, Z_FL_Q4, A_FL_Q4, B_FL_Q4, C_FL_Q4, X_Q5_Q6, X_Q6_EE, X_FL_RCM)
         */
        const toolDescriptionParameters TOOL_PARAMETERS{0.438, 0.0, 0.062, 0.0, 90.0, 0.0, 0.0088, 0.017, 0.305};

        /**
         * @fn calcDirKin
         * @brief Methode zur Berechnung der direkten Kinematik des Werkzeuges
         * @param Gelenkwinkel des Werkzeuges
         * @return Affine Transformation zwischen Flansch und Endeffektor
         */
        Eigen::Affine3d calcDirKin(Eigen::VectorXd);

        /**
         * @fn calcInvKin
         * @brief Methode zur Berechnung der inversen Kinematik des Werkzeuges auf Basis geometrischer Bedingungen
         * @param Endeffektorlage im Weltkoordinatensystem
         * @return Flag, ob der Arbeitsraum eingehalten wurde
         */
        bool calcInvKin(Eigen::Affine3d);

        /**
         * @fn rcmCallback
         * @brief Callback für den RCM-Service, indem der Trokarpunkt berechnet wird
         * @param req LBR-Startposition
         * @param resp Trokarpunkt im Weltkoordinatensystem
         * @return Flag, ob der Serviceaufruf erfolgreich war
         * @see GeometricKinematicRCM.srv
         */
        bool rcmCallback(masterslave::GeometricKinematicRCM::Request &req, masterslave::GeometricKinematicRCM::Response &resp);

        /**
         * @fn directKinematicsCallback
         * @brief Callback für den direkten Kinematik-Service, indem der TCP berechnet wird
         * @param req LBR-Flanschlage und Gelenkwinkel des Werkzeuges
         * @param resp Aktueller TCP im Weltkoordinatensystem
         * @return Flag, ob der Serviceaufruf erfolgreich war
         * @see GeometricKinematicDirectKinematic.srv
         */
        bool directKinematicsCallback(masterslave::GeometricKinematicDirectKinematics::Request &req, masterslave::GeometricKinematicDirectKinematics::Response &resp);

        /**
         * @fn inverseKinematicsCallback
         * @brief Callback für den inversen Kinematik-Service, inder die Flanschlage im Weltkoordinatensystem und die Gelenkwinkel des Werkzeuges berechnet werden
         * @param req Gewünschter TCP im Weltkoordinatensystem
         * @param resp LBR-Flanschlage im Weltkoordinatensystem und Gelenkwinkel des Werkzeuges
         * @return Flag, ob der Serviceaufruf erfolgreich war
         * @see GeometricKinematicInverseKinematic.srv
         */
        bool inverseKinematicsCallback(masterslave::GeometricKinematicInverseKinematics::Request &req, masterslave::GeometricKinematicInverseKinematics::Response &resp);

        /**
         * @var T_0_FL
         * @brief Flanschlage des Roboters im Weltkoordinatensystem
         */
        Eigen::Affine3d T_0_FL;

        /**
         * @var rcmServer
         * @brief ServiceServer für die Bestimmung des Trokarpunktes
         */
        ros::ServiceServer rcmServer;

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

        /**
         * @fn buildAffine3d
         * @brief Hilfsfunktion zur Erstellung von Affinen Transformationen aus einem Translationsvekt und den RPY-Winkeln
         * @param translXYZ Translationsvektor
         * @param axisZYX Rotationsvektor
         * @param zyx Flag zur Festlegung der Rotationsrichtung
         * @return Affine Transformation
         */
        Eigen::Affine3d buildAffine3d(const Eigen::Vector3d &translXYZ, const Eigen::Vector3d &axisZYX, bool zyx);


};

#endif // GeometricKinematic_H
