#ifndef TRAJECTORYGENERATOR_H
#define TRAJECTORYGENERATOR_H

#include "masterslave/Manipulation.h"
#include "ros/ros.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/Float64.h"

#include "Eigen/Core"

#include <eigen_conversions/eigen_msg.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include "masterslave/manipulation/trajectory/CircleTrajectory.h"
#include "masterslave/manipulation/trajectory/ITrajectory.h"
#include "masterslave/manipulation/trajectory/LineTrajectory.h"
#include "masterslave/manipulation/trajectory/PTPTrajectory.h"

#include <dynamic_reconfigure/server.h>
#include "masterslave/TrajectoryGeneratorConfig.h"

#include "staticFunctions.h"
/**
 * @enum TRAJECTORY_STATE
 * @brief Verschiedene Trajektorienarten, die die Klassen TrajectoryGenerator zur Verfügung stellt
 * @see TrajectoryGenerator
 */
enum TRAJECTORY_STATE
{
  NO_STATE = -1,
  PTP,
  LINE,
  CIRCLE
};

/**
 * @file TrajectoryGenerator.h
 *
 * @class TrajectoryGenerator
 * @brief Klasse zum Abfahren verschiedener Trajektorien zur quantitativen Bestimmung des Trokarpunktes
 *
 * @author Fabian Baier
 * @date 20.03.2016
 */

class TrajectoryGenerator
{
public:
  TrajectoryGenerator(ros::NodeHandle& nh);
  ~TrajectoryGenerator();

private:
  /**
   * @fn trajectoryCallback
   * @brief Callback der den aktuellen TCP bereitstellt und den neu bestimmten TCP als Antwort fordert
   * @param req TCP vor der Manipulation
   * @param resp TCP nach der Manipulation
   * @return Flag, ob der Serviceaufruf erfolgreich war
   */
  bool trajectoryCallback(masterslave::Manipulation::Request& req, masterslave::Manipulation::Response& resp);

  /**
   * @fn cycleTimeCallback
   * @brief Bereitstellung der Zykluszeit über ein ROS-Topic
   */
  void cycleTimeCallback(const std_msgs::Float64ConstPtr&);

  /**
   * @fn configurationCallback
   * @brief Dynamic Reconfigure Callbackmethode
   * @param config Dynamic Reconfigure Configuration
   * @param level Bitmaske
   * @see TrajectoryGenerator.cfg
   */
  void configurationCallback(masterslave::TrajectoryGeneratorConfig& config, uint32_t level);
  ros::NodeHandle nh_;

  /**
   * @var trajectoryServer
   * @brief Bereitstellung des Service Servers zur Manipulation des TCP
   */
  ros::ServiceServer trajectoryServer;

  /**
   * @var cycleTimeSub
   * @brief Empfänger in ROS für die bereitgestellte Zykluszeit
   */
  ros::Subscriber cycleTimeSub;

  /**
   * @var rcm
   * @brief Position des Trokarpunktes im Weltkoordinatensystem
   */
  Eigen::Vector3d rcm;

  /**
   * @var startPosition
   * @brief Aktuelle Startposition bzw. aktuelle TCP-Lage
   */
  Eigen::Affine3d startPosition;

  /**
   * @var firstPoint
   * @brief Erster Punkt der Trajektorie
   */
  Eigen::Vector3d firstPoint;

  /**
   * @var secondPoint
   * @brief Zweiter Punkt der Trajektorie
   */
  Eigen::Vector3d secondPoint;

  /**
   * @var zCoordinate
   * @brief z-Koordinate des Kreismittelpunktes
   */
  double zCoordinate;

  /**
   * @var circleRadius
   * @brief Radius der Kreistrajectorie
   */
  double circleRadius;

  /**
   * @var trajectorySpeed
   * @brief Durchschnittliche Trajektoriengeschwindigkeit in [mm/s]
   */
  double trajectorySpeed;

  /**
   * @var cycleTime
   * @brief Zykluszeit
   */
  double cycleTime;

  /**
   * @var start
   * @brief Startflag, die über Dynamic Reconfigure übertragen wird
   */
  bool start{ false };

  /**
   * @var rcmPositionThere
   * @brief Flag, ob der RCM empfangen wurde
   */
  bool rcmPositionThere{ false };

  const double MM_TO_M{ 0.001 };

  /**
   * @var trajectoryGen
   * @brief Zeiger auf das Trajectorienobjekt
   * @see ITrajectory
   */
  std::unique_ptr<ITrajectory> trajectoryGen;

  /**
   * @var state
   * @brief Aktuell gewünschte Trajektorienform
   */
  TRAJECTORY_STATE state{ NO_STATE };
};

#endif  // TRAJECTORYGENERATOR_H
