#ifndef MASTERSLAVEMANIPULATION_H
#define MASTERSLAVEMANIPULATION_H

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/TwistStamped.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include "Eigen/Core"

#include <eigen_conversions/eigen_msg.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include "masterslave/Manipulation.h"

#include "staticFunctions.h"

/**
 * @file MasterSlaveManipulation.h
 *
 * @class MasterSlaveManipulation
 * @brief Klasse, die sich um die Manipulation des TCP mittels des Eingabegerätes kümmert
 *
 * @author Fabian Baier
 * @date 20.03.2016
 */

class MasterSlaveManipulation
{
public:
  MasterSlaveManipulation(ros::NodeHandle& nh);

private:
  /**
   * @fn getControlDevice
   * @brief Die Methode liest das Eingabegerät vom Parameterserver aus
   */
  void getControlDevice();

  /**
   * @fn masterSlaveCallback
   * @brief Manipulation Service Callbackmethode
   * @param req Alter TCP vor der Manipulation
   * @param resp Neuer TCP nach der Manipulation
   * @return Flag, die den Erfolg des Service Calls anzeigt
   */
  bool masterSlaveCallback(masterslave::Manipulation::Request& req, masterslave::Manipulation::Response& resp);

  /**
   * @fn velocityCallback
   * @brief Callbackmethode in der die Achswerte des Eingabegerätes ausgelesen werden
   */
  void velocityCallback(const geometry_msgs::TwistStampedConstPtr&);

  /**
   * @fn cycleTimeCallback
   * @brief Callback zum Empfang der Zykluszeit
   */
  void cycleTimeCallback(const std_msgs::Float64ConstPtr&);

  ros::NodeHandle nh_;

  /**
   * @var masterSlaveServer
   * @brief Service Server zur Bereitstellung des Master-Slave-Service
   * @see MasterSlave.srv
   */
  ros::ServiceServer masterSlaveServer;

  /**
   * @var velocitySub
   * @brief Empfänger des Achsvektors des Eingabegerätes
   * @see velocityCallback
   */
  ros::Subscriber velocitySub;

  /**
   * @var cycleTimeSub
   * @brief Empfänger für die Zykluszeit
   * @see cycleTimeCallback
   */
  ros::Subscriber cycleTimeSub;

  /**
   * @var velocity
   * @brief Vorheriger Achsvektor des Eingabegerätes
   */
  geometry_msgs::TwistStamped velocity;

  /**
   * @var cycleTime
   * @brief Zykluszeit
   */
  double cycleTime;
};

#endif  // MASTERSLAVEMANIPULATION_H
