#ifndef BOUNDINGBOX_H
#define BOUNDINGBOX_H

#include <Eigen/Dense>
// nur fürs Debugging mit ROS
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>

/**
 * @file BoundingBox.h
 *
 * @class BoundingBox
 * @brief Arbeitsraumbegrenzung in Form eines Quaders
 *
 * @author Fabian Baier
 * @date 19.03.2016
 */

class BoundingBox
{
public:
  BoundingBox(ros::NodeHandle& nh, Eigen::Affine3d TCP, Eigen::Vector3d RemoteCenterOfMotion,
              Eigen::Vector3d boundingBoxSizeVector, double rcmDistance);
  void setTCPDistance(double value);
  void setBoundingBox(Eigen::Vector3d value);
  void setTCP(Eigen::Affine3d TCP);
  void setRCM(Eigen::Vector3d remoteCenterOfMotion);
  /**
   * @fn checkBoundingBoxTCP
   * @brief Überprüft, ob der TCP außerhalb des erlaubten Arbeitsraumes liegt
   * \param TCP aktueller TCP
   * @return Flag, ob der TCP im erlaubten Arbeitsraum liegt
   */
  bool checkBoundingBoxTCP(Eigen::Affine3d TCP);

private:
  ros::NodeHandle nh_;
  /**
   * @var centerPub
   * @brief Sender in ROS, um in V-Rep den Quader an der richtigen Stelle anzuzeigen
   */
  ros::Publisher centerPub;

  /**
   * @var RCM
   * @brief Koordinaten des Trokarpunktes
   */
  Eigen::Vector3d RCM;
  /**
   * @var TCP_old
   * @brief alter TCP
   */
  Eigen::Affine3d TCP_old;

  /**
   * @var rcmDistanceZ
   * @brief Abstand in z-Richtung zwischen dem Trokarpunkt und der Oberseite des Quaders
   */
  double rcmDistanceZ;

  /**
   * @var boundingBoxSize
   * @brief Größe des Quaders in drei Dimensionen
   */
  Eigen::Vector3d boundingBoxSize;

  /**
   * @var upperLeftCorner
   * @brief Koordinaten der oberen linken Ecke des Quaders
   */
  Eigen::Vector3d upperLeftCorner;

  /**
   * @var lowerRightCorner
   * @brief Koordinaten er linken unteren Ecke des Quaders
   */
  Eigen::Vector3d lowerRightCorner;

  /**
   * @var centerOfBoundingBox
   * @brief Zentroid des Quaders
   */
  Eigen::Vector3d centerOfBoundingBox;
};

#endif  // BOUNDINGBOX_H
