#ifndef VISUALSERVOING_H
#define VISUALSERVOING_H

#include "sensor_msgs/JointState.h"

#include "Eigen/Dense"
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "geometry_msgs/TwistStamped.h"
#include "masterslave/Manipulation.h"
#include "ros/ros.h"

#include <array>

#include <eigen_conversions/eigen_msg.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <dynamic_reconfigure/server.h>
#include "masterslave/VisualServoingConfig.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include "staticFunctions.h"

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

class VisualServoing
{
public:
  /**
   * @brief VisualServoing Default Konstruktor
   * @param nh ROS Node Handle für die ROS-Kommunikation
   */
  VisualServoing(ros::NodeHandle& nh);

private:
  void manipulateTransform(Eigen::Vector3d translation, Eigen::Vector3d orientation);

  void velocityCallback(const geometry_msgs::TwistStampedConstPtr val);

  void getControlDevice();

  void markerJointAngleCallback(const sensor_msgs::JointStateConstPtr& Q6Pstate,
                                const sensor_msgs::JointStateConstPtr& Q6Nstate);

  /**
   * @fn markerCallback
   * @brief Methode, welche die Markerlagen im Raum empfängt, die relevanten Markerlagen speichert und die
   * Differenztransformation bestimmt
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
   * @fn calculateTranslationalPIDcallback
   * @brief Berechnung des translatorischen PID-Regler-Rückgabewertes mit den Reglerverstärkungen @var pTrans, @var
   * dTrans und @var iTrans
   *
   * @return Reglerausgang für den translatorischen Anteil
   */
  Eigen::Vector3d calculateTranslationalPID();

  /**
   * @fn calculateRotationalPID
   * @brief Berechnung des translatorischen PID-Regler-Rückgabewertes mit den Reglerverstärkungen @var pRot, @var dRot
   * und @var iRot
   * @return Reglerausang für den rotatorischen Anteil
   */
  Eigen::Quaterniond calculateRotationalPID();

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

  ros::Subscriber velocitySub;

  ros::Publisher differenceTransformPub;

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

  Eigen::Affine3d lastTransform;
  /**
   * @var differenceTransform
   * @brief Transformationsdifferenz zwischen aktueller und ursprünglicher Transformation, die ausgeregelt werden muss
   * im Zyklus i
   */
  Eigen::Affine3d differenceTransform;

  /**
   * @var differenceTransformold
   * @brief Transformationsdifferenz zwischen aktueller und ursprünglicher Transformation, die ausgeregelt werden muss
   * im Zyklus i-1
   */
  Eigen::Affine3d differenceTransformOld;

  /**
   * @var ids
   * @brief IDs, die für das MarkerTracking relevant sind
   */
  std::array<int, 2> ids = { { 0, 1 } };
  bool initialRun = true;

  /**
   * @var pTrans
   * @var dTrans
   * @var iTrans
   * @brief Translatorische Reglerverstärkungen
   */
  double pTrans{ 0 };
  double dTrans{ 0.00 };

  /**
   * @var pRot
   * @var dRot
   * @var iRot
   * @brief Rotatorische Reglerverstärkungen
   */
  double pRot{ 0 };
  double dRot{ 0 };

  bool markerFoundTCP{ false };
  bool markerFoundObject{ false };

  ros::Subscriber markerJointAngleSub;

  double markerJointAngle{ 0 };

  const double DEG_TO_RAD{ M_PI / 180 };

  const double MM_TO_M{ 0.001 };

  const double MARKER_TO_TCP_ANGLE{ 9.53 * DEG_TO_RAD };

  Eigen::Affine3d markerJointRotation;

  double markerCycleTime;

  double lastMarkerTime;

  double cycleTime;

  double lastTime{ 0 };

  Eigen::Quaterniond initialRotationRobot;

  bool initialRunVisualServoing{ true };

  geometry_msgs::TwistStamped velocity;

  // Zum Togglen des InitialShot
  bool resetInitOld;

  Eigen::Affine3d tcpOld;

  Eigen::Affine3d objOld;

  double MINIMAL_DISTANCE{ 2e-3 };

  double MINIMAL_STEP_DISTANCE{ 2e-3 };

  Eigen::Affine3d lastTrackedPosition;

  dynamic_reconfigure::Server<masterslave::VisualServoingConfig> server;
};

#endif  // VISUALSERVOING_H
