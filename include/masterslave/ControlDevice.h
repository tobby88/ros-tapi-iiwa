#ifndef CONTROLDEVICE_H
#define CONTROLDEVICE_H

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <masterslave/ControlDeviceConfig.h>
#include <algorithm>
#include <map>
#include <sstream>
#include <string>
#include <string>
#include <vector>
#include "geometry_msgs/TwistStamped.h"
#include "masterslave/Button.h"
#include "sensor_msgs/Joy.h"

/**
 * \file ControlDevice.h
 *
 * \class ControlDevice
 * \brief Klasse zur Ansteuerung und Verwaltung verschiedener LowLevel-Eingabegeräte, wie GamePads oder dem
 * SpaceNavigator
 *
 * \author Fabian Baier
 * \date 19.03.2016
 *
 * \version 0.2
 */

class ControlDevice
{
public:
  ControlDevice(ros::NodeHandle& nh);

  /**
   * \fn void configurationCallback(masterslave::ControlDeviceConfig &config, uint32_t level)
   * @brief dynamic reconfigure Callback Methode
   * @param config Konfiguration für verschiedene Filterlevel und Verstärkungsfaktoren
   * @param level Bitmaske um Einstellungen zu maskieren
   * @see ControlDevice.cfg
   */
  void configurationCallback(masterslave::ControlDeviceConfig& config, uint32_t level);
  ~ControlDevice();

private:
  /**
   * @fn controlDeviceCallback
   * \brief Callback-Methode für die Eigabegeräte bzw. deren Achsen und Knöpfe
   * @param joy Achs- und Knopfvektoren des jeweiligen Eingabegeräts
   */
  void controlDeviceCallback(const sensor_msgs::Joy::ConstPtr& joy);
  /**
   * @fn registration
   * @brief Registrierung des Eingabegerätes mit Informationen des Parameterserver
   */
  void registration();

  /**
   * @fn buttonCheck
   * @brief Festlegung der Knopfbelegung
   */
  void buttonCheck();

  /**
   * @var errorShown
   * @brief Flag, welches anzeigt, ob der "Kein-Eingabegerät"-Fehler bereits gezeigt wird
   */
  bool errorShown;

  ros::NodeHandle globalNH_;
  ros::NodeHandle nh_;

  /**
   * @var deviceSub
   * @brief Empfänger in ROS, der die Daten der Eingabegeräte empfängt
   */
  ros::Subscriber deviceSub;

  /**
   * @var axisPub
   * @brief Sender in ROS, der die gefilterten und skalierten Achswerte weitersendet
   */
  ros::Publisher axisPub;

  /**
   * @var buttonsPub
   * @brief Sender in ROS, der die Knöpfe sendet
   */
  ros::Publisher buttonsPub;

  /**
   * @var buttons
   * @brief Buttons und Funktionen sind in dieser Map verknüpft
   */
  std::map<int, std::string> buttons;

  /**
   * @var rotGain
   * @var transGain
   * @brief Verstärkungen für die Rotations- und Translationsachsen
   */
  double rotGain;
  double transGain;

  /**
   * @var joyThresh
   * @brief Schwellwert für die Joystickachsen
   */
  double joyThresh;

  /**
   * @var curDeviceType
   * @brief aktueller Eingabegerätetyp (GAMEPAD, SPACENAV)
   */
  std::string curDeviceType;

  std::string apiDevice;

  // number of the device
  int curDeviceNum;

  /**
   * @var joy_old
   * @brief Joystickachswerte des letzten Zyklus, um Änderungen detektieren zu können
   */
  sensor_msgs::Joy joy_old;
};

#endif  // CONTROLDEVICE_H
