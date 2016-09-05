#ifndef STATICFUNCTIONS_H
#define STATICFUNCTIONS_H

#include "Eigen/Dense"

#define DEG M_PI / 180.0f

/**
 * @fn QuaternionFromEuler
 * @brief Hilfsmethode um ein Quaternion aus Eulerwinkel zu erstellen
 * @param eulerXYZ Eulerwinkel (Reihenfolge im Vektor X-Y-Z)
 * @param ZYX Flag, ob die Rotationsreihenfolge X-Y'-Z'' oder Z-Y'-X'' ist
 * @return Das berechnete Quaternion
 */

Eigen::Quaternion<double> QuaternionFromEuler(const Eigen::Vector3d &eulerXYZ, bool ZYX = true)
{
  Eigen::Quaternion<double> quat;
  quat.Identity();
  Eigen::AngleAxisd zAngle(eulerXYZ[2], Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd yAngle(eulerXYZ[1], Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd xAngle(eulerXYZ[0], Eigen::Vector3d::UnitX());
  if (ZYX)
    quat = zAngle * yAngle * xAngle;
  else
    quat = xAngle * yAngle * zAngle;

  return quat;
}

/**
 * @fn rotation2RPY
 * @brief Umrechnung einer Rotation einer Affinen Transformation in Euler-Winkel und Übergabe der Translation
 * @param transformation Transformation
 * @return Vektor mit Translation und Rotation in Euler-Winkeln
 */

Eigen::VectorXd rotation2RPY(Eigen::Affine3d transformation)
{
  Eigen::VectorXd retVal = Eigen::VectorXd::Zero(6);
  // Conversion from Transformation Matrix into Transformation Vector and Euler Angles YPR
  retVal.head(3) = transformation.translation();
  retVal(3) = atan2(transformation.matrix()(2, 1), transformation.matrix()(2, 2));
  retVal(5) = atan2(transformation.matrix()(1, 0), transformation.matrix()(0, 0));
  retVal(4) = atan2(-transformation.matrix()(2, 0),
                    sqrt(pow(transformation.matrix()(0, 0), 2) + pow(transformation.matrix()(1, 0), 2)));
  return retVal;
}

/**
 * @fn buildAffine3d
 * @brief Hilfsfunktion zur Erstellung von Affinen Transformationen aus einem Translationsvekt und den RPY-Winkeln
 * @param translXYZ Translationsvektor
 * @param axisZYX Rotationsvektor
 * @param zyx Flag zur Festlegung der Rotationsrichtung
 * @return Affine Transformation
 */

Eigen::Affine3d buildAffine3d(const Eigen::Vector3d &translXYZ, const Eigen::Vector3d &axisZYX, bool zyx = true)
{
  Eigen::Affine3d transl;
  transl.setIdentity();
  transl.translate(translXYZ);
  if (zyx)
  {
    transl.rotate(Eigen::AngleAxis<double>(axisZYX(2), Eigen::Vector3d::UnitZ()));
    transl.rotate(Eigen::AngleAxis<double>(axisZYX(1), Eigen::Vector3d::UnitY()));
    transl.rotate(Eigen::AngleAxis<double>(axisZYX(0), Eigen::Vector3d::UnitX()));
  }
  else
  {
    transl.rotate(Eigen::AngleAxis<double>(axisZYX(0), Eigen::Vector3d::UnitX()));
    transl.rotate(Eigen::AngleAxis<double>(axisZYX(1), Eigen::Vector3d::UnitY()));
    transl.rotate(Eigen::AngleAxis<double>(axisZYX(2), Eigen::Vector3d::UnitZ()));
  }
  return transl;
}

Eigen::Affine3d filterPose(Eigen::Affine3d actualPose, Eigen::Affine3d lastPose, const double MINIMAL_DISTANCE,
                           const double MINIMAL_STEP_DISTANCE)
{
  Eigen::Affine3d newPose = Eigen::Affine3d::Identity();
  Eigen::Affine3d differencePose = actualPose * lastPose.inverse();
  double differenceDistance = differencePose.translation().norm();
  // Fals die minimale Distanz unterschritten wird, passiert nichts.
  ROS_INFO_STREAM("differenceDistance: " << differenceDistance);
  if (differenceDistance >= MINIMAL_DISTANCE)
  {
    double minimalDistanceFactor = std::abs((differenceDistance - MINIMAL_DISTANCE) /
                                            (differenceDistance - MINIMAL_DISTANCE + MINIMAL_STEP_DISTANCE));
    ROS_WARN_STREAM("Test" << minimalDistanceFactor);
    if (minimalDistanceFactor > 1)
      minimalDistanceFactor = 1;
    newPose.translate(lastPose.translation() + differencePose.translation() * minimalDistanceFactor);
  }
  else
  {
    newPose.translate(lastPose.translation());
  }

  Eigen::AngleAxisd differenceAngle = Eigen::AngleAxisd(differencePose.rotation());
  Eigen::Quaterniond lastPoseQuat = Eigen::Quaterniond(lastPose.rotation());
  Eigen::Quaterniond actualPoseQuat = Eigen::Quaterniond(actualPose.rotation());
  if (differenceAngle.angle() >= 2 * DEG)
  {
    double minimalAngleFactor =
        std::abs((differenceAngle.angle() - 2 * DEG) / (differenceAngle.angle() - 2 * DEG + 3 * DEG));
    if (minimalAngleFactor > 1)
      minimalAngleFactor = 1;
    newPose.rotate(lastPoseQuat.slerp(minimalAngleFactor, actualPoseQuat));
  }
  else
  {
    newPose.rotate(lastPoseQuat);
  }
  return newPose;
}

Eigen::Affine3d checkAngle(Eigen::Affine3d newPose, Eigen::Affine3d oldPose)
{
  Eigen::Affine3d returnValue;
  Eigen::Quaterniond newRot(newPose.rotation());
  Eigen::Quaterniond oldRot(oldPose.rotation());

  Eigen::AngleAxisd difRot = Eigen::AngleAxisd(newRot.inverse() * oldRot);

  if (std::abs(difRot.angle()) < (double)5 / 180 * M_PI)
  {
    returnValue = newPose;
  }
  else if (std::abs(difRot.angle()) <= (double)15 / 180 * M_PI)
  {
    returnValue.translate(oldPose.translation());
    returnValue.rotate((oldRot.slerp((double)5 / 180 * M_PI / difRot.angle(), newRot)));
    ROS_WARN("Big rotational Difference");
  }
  else if (std::abs(difRot.angle()) > (double)15 / 180 * M_PI)
  {
    ROS_ERROR_STREAM("Rotation too big!!: " << difRot.angle() << " " << (double)15 / 180 * M_PI);
    returnValue = oldPose;
  }
  return returnValue;
}

#endif  // STATICFUNCTIONS_H
