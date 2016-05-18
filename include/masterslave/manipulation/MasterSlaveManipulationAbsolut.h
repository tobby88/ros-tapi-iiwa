#ifndef MASTERSLAVEMANIPULATIONABSOLUT_H
#define MASTERSLAVEMANIPULATIONABSOLUT_H

#include <algorithm>
#include <array>

#include "ros/ros.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "masterslave/Manipulation.h"

#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float64.h"
#include "Eigen/Dense"

#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>

#include <dynamic_reconfigure/server.h>
#include "masterslave/MasterSlaveManipulationAbsoluteConfig.h"

/**
 * @file MasterSlaveManipulationAbsolute.h
 *
 * @class MasterSlaveManipulationAbsolute
 * @brief Klasse, die sich um die Manipulation des TCP mittels des Markertrackings kümmert
 *
 * @author Fabian Baier
 * @date 04.04.2016
 */

class MasterSlaveManipulationAbsolute
{
public:

    /**
     * @fn MasterSlaveManipulationAbsolute
     * @brief Standardkonstruktor
     * @param nh ROS-Node Handle
     */
    MasterSlaveManipulationAbsolute(ros::NodeHandle &nh);
private:
    /**
     * @fn markerCallback
     * @brief gemeinsamer Callback der beiden MarkerTrackingInstanzen für das Tracking der Referenz- und der Steuerungsmarker
     * @param handMarker Steuerungsmarkervektor
     * @param referenceMarker Referenzmarkervektor
     */
    void markerCallback(const ar_track_alvar_msgs::AlvarMarkersConstPtr &handMarker);
    /**
     * @fn cycleTimeCallback
     * @brief Zykluszeit der Interpolation bzw. der inversen Kinematik
     * @param val Zykluszeit
     */
    void cycleTimeCallback(const std_msgs::Float64ConstPtr& val);

    /**
     * @fn masterSlaveCallback
     * @brief Implementierung des MasterSlaveServices zur Steuerung des Roboters
     * @param req Request des Clients: alte Transformation vor der Manipulation
     * @param resp Response des Servers: neue Transformation nach der Manipulation
     * @return Status, ob die Manipulation erfolgreich war
     */
    bool masterSlaveCallback(masterslave::Manipulation::Request& req, masterslave::Manipulation::Response& resp);


    void configurationCallback(masterslave::MasterSlaveManipulationAbsoluteConfig &config, uint32_t level);

    /**
     * @var nh_
     * @brief ROS-NodeHandle
     */
    ros::NodeHandle nh_;

    ros::Publisher pliersDistancePub;

    /**
     * @var markerSub
     * @brief Empfänger für die Steuerungsmarker
     */
    ros::Subscriber markerSub;



    /**
     * @var cycleTimeSub
     * @brief Empfänger für die Zykluszeit
     */
    ros::Subscriber cycleTimeSub;

    /**
     * @var masterSlaveServer
     * @brief Serverinstanz für die MasterSlaveSchnittstelle
     */
    ros::ServiceServer masterSlaveServer;

    /**
     * @var cycleTime
     * @brief Zykluszeit
     */
    double cycleTime;

    Eigen::Affine3d poseIndexFinger;

    Eigen::Affine3d poseThumb;

    /**
     * @var poseAct
     * @brief aktuelle Lage des TCP
     */
    Eigen::Affine3d poseAct;

    /**
     * @var poseOld
     * @brief Lage des TCP im vorherigen Frame
     */
    Eigen::Affine3d initialPoseMarker;

    /**
     * @var difference
     * @brief Differenztransformation zwuischen der alten und aktuellen TCP-Lage
     */
    Eigen::Vector3d difference;

    /**
     * @var initialRun
     * @brief Initialer Durchlauf (ja oder nein)
     */
    bool initialRun{true};

    bool initialRunMasterSlave{true};

    /**
     * @var frameTime
     * @brief Zeit zwischen zwei Frames der Kamera
     */
    double frameTime;

    /**
     * @var lastTime
     * @brief Zeitpunkt des letzten Frames in Sekunden
     */
    double lastFrameTime;

    /**
     * @var masterSlaveTime
     * @brief Zeit zwischen zwei MasterSlaveCallbacks
     */

    double masterSlaveTime;

    /**
     * @var lastMasterSlaveTime
     * @brief Letzter Aufruf des MasterSlaveCallbacks
     */

    double lastMasterSlaveTime;

    /**
     * @var slerpParameter
     * @brief Parameter [0;1] für die Rotationsinterpolation
     */
    double slerpParameter{0};

    /**
     * @var handMarkerFound
     * @brief Wurden die Steuerungsmarker gefunden?!
     * @see masterSlaveCallback
     */
    bool indexFingerMarkerFound{false};

    bool thumbMarkerFound{false};

    /**
     * @var referenceMarkerFound
     * @brief Wurden die Referenzmarker gefunden?!
     * @see masterSlaveCallback
     */
    bool referenceMarkerFound{false};

    /**
     * @var markerCallbackCalled
     * @brief Wurde der MarkerCallback bereits einmal ausgelöst?
     * @see masterSlaveCallback
     */
    bool markerCallbackCalled{false};

    Eigen::Quaterniond initialRotationMarker;

    Eigen::Quaterniond initialRotationRobot;

    Eigen::Quaterniond oldRotation;

    Eigen::Vector3d initialPoseRobot;

    double transMotionScaling{1};

    double rotationScaling{1};

    const double MINIMAL_DISTANCE{4e-03};

    const double MINIMAL_STEP_DISTANCE{2e-03};

    const double MAXIMUM_DIFFERENCE_ANGLE_PER_STEP{M_PI/4};

    const double MAXIMUM_TRANSLATIONAL_VELOCITY{0.1};



};

#endif // MASTERSLAVEMANIPULATIONABSOLUT_H
