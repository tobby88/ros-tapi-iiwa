#ifndef VISUALSERVOING_H
#define VISUALSERVOING_H

#include "ros/ros.h"
#include "Eigen/Dense"
#include "masterslave/Manipulation.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"

#include <array>

#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>

#include <dynamic_reconfigure/server.h>
#include "masterslave/VisualServoingConfig.h"

/**
 * @file VisualServoing.h
 *
 * @class VisualServoing
 * @brief Klasse zur Bedienung der Manipulationsschnittstelle und Anbindung des Marker Trackings mit ar_track_alvar
 *
 * @author Fabian baier
 * @date 31.03.2016
 *
 * @link http://wiki.ros.org/ar_track_alvar
 */

class VisualServoing {
public:
    /**
     * @brief VisualServoing Default Konstruktor
     * @param nh ROS Node Handle für die ROS-Kommunikation
     */
    VisualServoing(ros::NodeHandle &nh);
private:

    /**
     * @fn markerCallback
     * @brief Methode, welche die Markerlagen im Raum empfängt, die relevanten Markerlagen speichert und die Differenztransformation bestimmt
     * @param markerPtr Zeiger auf das Markerarray
     */
    void markerCallback(const ar_track_alvar_msgs::AlvarMarkersConstPtr& markerPtr);

    /**
     * @fn configurationCallback
     * @brief Dynamic Reconfigure Callbackmethode
     * @param config Struktur mit den verwendeten Parametern
     * @param level Bitmaske
     * @see VisualServoing.cfg
     */
    void configurationCallback(masterslave::VisualServoingConfig& config, uint32_t level);

    /**
     * @fn visualServoingCallback
     * @brief Callbackmethode zur Bereitstellung der Manipulationsschnittstelle
     * @param req Alte Transformation von der Basis zum TCP
     * @param resp Neue manipulierte Transformation von der Roboterbasis zum TCP
     * @return Flag, ob der Servicecall erfolgreich war
     * @see Manipulation.srv
     */
    bool visualServoingCallback(masterslave::Manipulation::Request& req, masterslave::Manipulation::Response& resp);

    /**
     * @fn calculateTranslationalPID
     * @brief Berechnung des translatorischen PID-Regler-Rückgabewertes mit den Reglerverstärkungen @var pTrans, @var dTrans und @var iTrans
     *
     * @return Reglerausgang für den translatorischen Anteil
     */
    Eigen::Vector3d calculateTranslationalPID();

    /**
     * @fn calculateRotationalPID
     * @brief Berechnung des translatorischen PID-Regler-Rückgabewertes mit den Reglerverstärkungen @var pRot, @var dRot und @var iRot
     * @return Reglerausang für den rotatorischen Anteil
     */
    Eigen::Matrix3d calculateRotationalPID();

    ros::NodeHandle nh_;

    /**
     * @var markerSub
     * @brief Empfänger der Markerlagen im Raum über ROS
     */
    ros::Subscriber markerSub;

    /**
     * @var visualServoingServer
     * @brief Server zur Bereitstellung der Manipulationsschnittstelle
     */
    ros::ServiceServer visualServoingServer;

    /**
     * @var initialTransform
     * @brief ursprüngliche Transformation zwischen TCP und verfolgtem Objekt
     */
    Eigen::Affine3d initialTransform;

    /**
     * @var actualTransform
     * @brief aktuelle Transformation zwischen TCP und verfolgtem Objekt
     */
    Eigen::Affine3d actualTransform;

    /**
     * @var differenceTransform
     * @brief Transformationsdifferenz zwischen aktueller und ursprünglicher Transformation, die ausgeregelt werden muss im Zyklus i
     */
    Eigen::Affine3d differenceTransform;

    /**
     * @var differenceTransformold
     * @brief Transformationsdifferenz zwischen aktueller und ursprünglicher Transformation, die ausgeregelt werden muss im Zyklus i-1
     */
    Eigen::Affine3d differenceTransformOld;

    /**
     * @var ids
     * @brief IDs, die für das MarkerTracking relevant sind
    std::array<int,2> ids = {{0,1}};
    bool initialRun = true;

    double cycleTime{0.033};

    /**
     * @var pTrans
     * @var dTrans
     * @var iTrans
     * @brief Translatorische Reglerverstärkungen
     */
    double pTrans{1};
    double dTrans{0.001};
    double iTrans{0.0001};

    /**
     * @var integralErrorTrans
     * @brief Integrierervariable für den translatorischen Anteil
     */
    Eigen::Vector3d integralErrorTrans{0,0,0};

    /**
     * @var pRot
     * @var dRot
     * @var iRot
     * @brief Rotatorische Reglerverstärkungen
     */
    double pRot{1};
    double dRot{0};
    double iRot{0};

    /**
     * @var integralErrorRot
     * @brief Integrierervariable für den rotatorischen Anteil
     */
    Eigen::Vector3d integralErrorRot;

};

#endif // VISUALSERVOING_H
