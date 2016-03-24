#ifndef NUMERICKINEMATIC_H
#define NUMERICKINEMATIC_H
#include "IKinematic.h"

#include <eigen3/Eigen/Core>
#include "ros/ros.h"


//service definitions
#include "masterslave/NumericKinematicRCM.h"
#include "masterslave/NumericKinematicDirectKinematics.h"
#include "masterslave/NumericKinematicInverseKinematics.h"

#include "masterslave/NumericKinematicConfig.h"


#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float64.h>

//solver algorithm for quadratic programming problems (very nice perfomance)
#include "eiquadprog.hpp"

/**
 * @file NumericKinematic.h
 *
 * @class NumericKinematic
 * @brief Klasse zur Berechnung der Kinematiken des Gesamtsystems mit 10 Achsen. Die inverse Kinematik wird numerisch mittels eines Least-Square-Problems gelöst
 * Die Klasse berechnet die direkte und inverse Kinematik eines Gesamtsystems bestehend aus LBR iiwa und dem Laparoskop. Es besitzt insgesamt 10 Achsen, da der Trokarpunkt eingehalten werden muss,
 * bleibt noch eine Redundanz mit Dimension 2 zur Optimierung der Gelenkwinkelkonfiguration.
 * Diese können mit verschiedenen Optimierungskriterien, wie Singularitätsvermeidung, Kollisionsvermeidung, Geschwindigkeits- und Beschleunigungsminimierung, sowie einer Winkelwächterfunktion parametriert werden
 * Zum Lösen wird der Algorithmus aus QuadProg++ eingesetzt, der auf die Eigen-Bibliothek umgeschrieben wurde
 * @see eiquadprog.hpp
 *
 * @author Fabian Baier
 * @date 20.03.2016
 * @see IKinematic
 */

class NumericKinematic: public IKinematic
{
public:
    NumericKinematic(ros::NodeHandle&);
private:

    ros::NodeHandle nh_;

    /**
     * @fn calcDirKin
     * @brief Berechnung der direkten Kinematik der Gesamtkinematik mit 10 Gelenkfreiheitsgraden
     * @param jointAngles Gelenkwinkelvektor
     * @return Kartesische Position und Vektor der Eulerwinkel in Z-Y'-X''-Reihenfolge
     */
    Eigen::VectorXd calcDirKin(Eigen::VectorXd jointAngles);

    /**
     * @fn calcAnalyticalJacobian
     * @brief Berechnung der analytischen Jacobi-Matrix der Gesamtkinematik mit 10 Gelenkfreiheitsgraden
     * @param jointAngles Gelenkwinkelvektor
     * @return Analytische Jacobi-Matrix mit dem rotatorischen Anteil in Bezug auf Eulerwinkel
     */
    Eigen::MatrixXd calcAnalyticalJacobian(Eigen::VectorXd jointAngles);

    /**
     * @fn calcInvKin
     * @brief Berechnung der inversen Kinematik durch numerische Optimierung eines Least-Squares-Problem nach verschiedenen Kriterien
     * @param T_0_EE gewünschte TCP-Lage
     * @return Flag, ob der definierte Arbeitsraum eingehalten wurde
     */
    bool calcInvKin(Eigen::Affine3d T_0_EE);

    // Methoden, welche die Kriterien für die Optimierungsbedingungen definieren

    /**
     * @fn angleMonitoring
     * @brief Überwacht die Einhaltung der maximalen Winkel und berechnet ein winkelabhängiges Potential
     * @param deltaQ Winkeldifferenz zwischen dem vorherigen Iterationsschritt und dem in dem vorherigen Durchlauf bestimmten Gelenkwinkelvektor
     * @param q Aktueller Gelenkwinkelvektor der vorherigen Iteration
     * @param Hmax Maximales Potential der Potentialfunktion
     * @param a Referenz auf das berechnete Potential
     * @return Ableitung des Potentials nach den Gelenkwinkeln
     */
    Eigen::VectorXd angleMonitoring(Eigen::VectorXd deltaQ, Eigen::VectorXd q, double Hmax, double& a);
    Eigen::MatrixXd collisionControl(Eigen::VectorXd q);

    /**
     * @fn trocarMonitoring
     * @brief Überwacht die Einhaltung des Trokarpunktes
     * @param qAct Aktueller Gelenkwinkelvektor der vorherigen Iteration
     * @param deltaQ Winkeldifferenz zwischen dem vorherigen Iterationsschritt und dem in dem vorherigen Durchlauf bestimmten Gelenkwinkelvektor
     * @param A Jacobi-Matrix der Potentialfunktion
     * @return Vektor der freiheitsgradabhängigen Potentiale
     */
    Eigen::VectorXd trocarMonitoring(Eigen::VectorXd qAct, Eigen::VectorXd deltaQ, Eigen::MatrixXd& A); // and RCM is needed

    /**
     * @fn minimizeVelocities
     * @brief Minimiert und verteilt die Gelenkwinkelgeschwindigkeiten abhängig von der Wichtungsmatrix
     * @param cycleTime Zykluszeit
     * @param weightMatrix Wichtungsmatrix
     * @return Vektor der Gelenkwinkelpotentiale abgeleitet nach den Gelenkwinkeln
     */
    Eigen::MatrixXd minimizeVelocities(double cycleTime, Eigen::MatrixXd weightMatrix);

    /**
     * @fn minimizeAcceleration
     * @brief Minimiert und verteilt die Gelenkwinkelbeschleunigungen abhängig von der Wichtungsmatrix
     * @param cycleTime Zykluszeit
     * @param weightMatrix Wichtungsmatrix
     * @param deltaQ Winkeldifferenz zwischen dem vorherigen Iterationsschritt und dem in dem vorherigen Durchlauf bestimmten Gelenkwinkelvektor
     * @param a Referenz auf das Potential
     * @return Vektor der Gelenkwinkelpotentiale abgeleitet nach den Gelenkwinkeln
     */
    Eigen::MatrixXd minimizeAcceleration(double cycleTime, Eigen::MatrixXd weightMatrix, Eigen::VectorXd deltaQ, Eigen::MatrixXd& a);

    /**
     * @fn avoidSingularities
     * @brief Vermeidet Singularitäten durch ein Kosinusquadratpotential
     * @param qAct Aktueller Gelenkwinkelvektor
     * @param qOffset Verschiebung des Kosinusterms
     * @param weight Maximales Potential
     * @param a Referenz auf das Potential
     * @return Vektor der Gelenkwinkelpotentiale abgeleitet nach den Gelenkwinkeln
     */
    Eigen::VectorXd avoidSingularities(Eigen::VectorXd qAct, Eigen::VectorXd qOffset, double weight, double& a);

    ros::Subscriber cycleTimeSub;

    //Callbacks
    bool rcmCallback(masterslave::NumericKinematicRCM::Request&, masterslave::NumericKinematicRCM::Response&);
    bool directKinematicsCallback(masterslave::NumericKinematicDirectKinematics::Request&, masterslave::NumericKinematicDirectKinematics::Response&);
    bool inverseKinematicsCallback(masterslave::NumericKinematicInverseKinematics::Request&, masterslave::NumericKinematicInverseKinematics::Response&);
    void cycleTimeCallback(const std_msgs::Float64ConstPtr&);
    void configurationCallback(masterslave::NumericKinematicConfig& config, uint32_t level);

    /**
     * @var LBR_PARAMETERS
     * @brief Geometrische Parameter des LBR iiwa R820
     * @see DescriptionParameters.h
     */
    const lbrDescriptionParameters LBR_PARAMETERS = { 0.160, 0.200, 0.200, 0.220, 0.180, 0.220, 0.076, 0.050};

    /**
     * @var TOOL_PARAMETERS
     * @brief Geoemtrische Parameter des Werkzeuges
     * @see DescriptionParameters.h
     */
    const toolDescriptionParameters TOOL_PARAMETERS = {0.438, 0.0, 0.062, 0.0, 90.0, 0.0, 0.0088, 0.017, 0.305};

    /**
     * @var URSULA_MAX_ANGLES
     * @brief Vektor der maximalen Gelenkwinkel
     * @see MAX_ANGLES
     */
    Eigen::Matrix<double, 10, 1> URSULA_MAX_ANGLES;

    /**
     * @var URSULA_MAX_ANGLES_SPEED
     * @brief Vektor der maximalen Gelenkwinkelgeschwindigkeiten
     * @see MAX_ANGLES_SPEED
     */
    Eigen::Matrix<double, 10, 1> URSULA_MAX_ANGLES_SPEED;

    /**
     * @var MAX_ANGLES
     * @brief Array der maximalen Gelenkwinkel in [rad]
     */
    const double MAX_ANGLES[10] = {170*DEG_TO_RAD, 120*DEG_TO_RAD, 170*DEG_TO_RAD, 120*DEG_TO_RAD, 170*DEG_TO_RAD, 120*DEG_TO_RAD, 175*DEG_TO_RAD, 85*DEG_TO_RAD, 90*DEG_TO_RAD, 90*DEG_TO_RAD};

    /**
     * @var MAX_ANGLES_SPEED
     * @brief Array der maximalen Gelenkwinkelgeschwindigkeiten in [rad/s]
     */
    const double MAX_ANGLES_SPEED[10] = { 85*DEG_TO_RAD, 85*DEG_TO_RAD, 100*DEG_TO_RAD, 75*DEG_TO_RAD, 130*DEG_TO_RAD, 135*DEG_TO_RAD, 135*DEG_TO_RAD, 135*DEG_TO_RAD, 135*DEG_TO_RAD, 135*DEG_TO_RAD};

    /**
     * @var jointWeightMatrix
     * @brief Diagonale Gelenkwinkelwichtungsmatrix [10x10]
     */
    Eigen::Matrix<double, 10, 10> jointWeightMatrix;

    /**
     * @var minDistance
     * @brief Distanz ab der die Kollisionserkennung startt
     */
    const double minDistance{0.05}; // minimal distance between some objects before collision control starts working


    /**
     * @var maxIterations
     * @brief Maximale Anzahl der Iterations
     */
    const int maxIterations{10};

    /**
     * @var cycleTime
     * @brief Zykluszeit
     */
    double cycleTime;

    //Endeffector Position in translation and rotation in euler angles (DLR-Convention)

    /**
     * @var curEEPosition
     * @brief Aktuelle Endeffektorlage im Weltkoordinatensystem
     */
    Eigen::Matrix<double, 6, 1> curEEPosition;

    /**
     * @var desEEPosition
     * @brief Gewünschte Endeffektorlage im Weltkoordinatensystem
     */
    Eigen::Matrix<double, 6, 1> desEEPosition;

    /**
     * @var T_0_Q1
     * @brief Lage des Gelenks Q1 im Weltkoordinatensystem
     */
    Eigen::Affine3d T_0_Q1;

    /**
     * @var T_0_Q2
     * @brief Lage des Gelenks Q2 im Weltkoordinatensystem
     */
    Eigen::Affine3d T_0_Q2;

    /**
     * @var T_0_Q3
     * @brief Lage des Gelenks Q3 im Weltkoordinatensystem
     */
    Eigen::Affine3d T_0_Q3;

    /**
     * @var T_0_Q4
     * @brief Lage des Gelenks Q4 im Weltkoordinatensystem
     */
    Eigen::Affine3d T_0_Q4;

    /**
     * @var T_0_Q5
     * @brief Lage des Gelenks Q5 im Weltkoordinatensystem
     */
    Eigen::Affine3d T_0_Q5;

    /**
     * @var T_0_Q6
     * @brief Lage des Gelenks Q6 im Weltkoordinatensystem
     */
    Eigen::Affine3d T_0_Q6;

    /**
     * @var T_0_Q7
     * @brief Lage des Gelenks Q7 im Weltkoordinatensystem
     */
    Eigen::Affine3d T_0_Q7;

    /**
     * @var T_0_FL
     * @brief Lage des Roboterflanschs im Weltkoordinatensystem
     */
    Eigen::Affine3d T_0_FL;

    /**
     * @var T_0_Q1
     * @brief Lage des Schafts im Weltkoordinatensystem
     */
    Eigen::Affine3d T_0_SCH;

    /**
     * @var T_0_Q8
     * @brief Lage des Gelenks Q8 im Weltkoordinatensystem
     */
    Eigen::Affine3d T_0_Q8;

    /**
     * @var T_0_Q9
     * @brief Lage des Gelenks Q9 im Weltkoordinatensystem
     */
    Eigen::Affine3d T_0_Q9;

    /**
     * @var T_0_Q10
     * @brief Lage des Gelenks Q10 im Weltkoordinatensystem
     */
    Eigen::Affine3d T_0_Q10;

    /**
     * @var collisionAvoidanceGain
     * @see collisionAvoidance
     */
    double collisionAvoidanceGain{0.0000001};

    /**
     * @var angleMonitoringGain
     * @see angleMonitoring
     */
    double angleMonitoringGain{0.0000001};

    /**
     * @var singularityGain
     * @see singularityAvoidance
     */
    double singularityGain{0.000001};

    /**
     * @var accelerationGain
     * @see accelerationMinimization
     */
    double accelerationGain{0.0000001};

    /**
     * @var velocityGain
     * @see veloticyMinimization
     */
    double velocityGain{0.0000001};

    /**
     * @var maxSpeed
     * @brief maxSpeed decimal
     */
    double maxSpeed{0.7};

    /**
     * @var rescueFactor
     * @deprecated
     */
    double rescueFactor{0};

    /**
     * @var trocarGain
     * @brief Gewichtung Einhaltung des Trokarpunktes
     */
    double trocarGain{1};

    /**
     * @var tcpGain
     * @brief Gewichtung Einhaltung des TCP
     */
    double tcpGain{1};

    /**
     * @var geomJacobian
     * @brief geometrische Jacobi-Matrix
     * @see calculateAnalyticalJacobian
     */
    Eigen::MatrixXd geomJacobian;

    /**
     * @var analyticalJacobian
     * @brief analytische Jacobi-Matrix
     * @see calculateAnalyticalJacobian
     */
    Eigen::MatrixXd analyticalJacobian;

    /**
     * @fn rotation2RPY
     * @brief Umrechnung einer Rotation einer Affinen Transformation in Euler-Winkel und Übergabe der Translation
     * @param transformation Transformation
     * @return Vektor mit Translation und Rotation in Euler-Winkeln
     */
    Eigen::VectorXd rotation2RPY(Eigen::Affine3d transformation);

    /**
     * @fn buildAffine3d
     * @brief Hilfsfunktion zur Erstellung von Affinen Transformationen aus einem Translationsvekt und den RPY-Winkeln
     * @param translXYZ Translationsvektor
     * @param axisZYX Rotationsvektor
     * @param zyx Flag zur Festlegung der Rotationsrichtung
     * @return Affine Transformation
     * @see IKinematic
     */
    Eigen::Affine3d buildAffine3d(const Eigen::Vector3d &translXYZ, const Eigen::Vector3d &axisZYX, bool zyx);
};

#endif // NumericKinematic_H
