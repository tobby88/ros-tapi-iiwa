#include "masterslave/commander/BoundingBox.h"

BoundingBox::BoundingBox(ros::NodeHandle& nh, Eigen::Affine3d TCP, Eigen::Vector3d RemoteCenterOfMotion,
                         Eigen::Vector3d boundingBoxSizeVector, double rcmDistance)
  : nh_(nh)
{
  TCP_old = TCP;
  RCM = RemoteCenterOfMotion;
  boundingBoxSize = boundingBoxSizeVector;
  rcmDistanceZ = rcmDistance;
  upperLeftCorner = RCM + Eigen::Vector3d(boundingBoxSize(0) / 2, boundingBoxSize(1) / 2, -rcmDistanceZ);
  lowerRightCorner = upperLeftCorner - boundingBoxSize;
  centerOfBoundingBox = RCM - Eigen::Vector3d(0, 0, rcmDistanceZ + boundingBoxSize(2) / 2);
  centerPub = nh_.advertise<geometry_msgs::Point>("/vrep/CenterOfBoundingBox", 1);

  geometry_msgs::Point center;
  tf::pointEigenToMsg(centerOfBoundingBox, center);
  centerPub.publish(center);
}

bool BoundingBox::checkBoundingBoxTCP(Eigen::Affine3d TCP)
{
  geometry_msgs::Point center;
  tf::pointEigenToMsg(centerOfBoundingBox, center);
  centerPub.publish(center);
  Eigen::Affine3d velocity = TCP_old.inverse() * TCP;
  for (int i = 0; i < 3; i++)
  {
    if (TCP.translation()(i) > upperLeftCorner[i] && velocity.translation()(i) > 0)
    {
      return false;
    }
    else if (TCP.translation()(i) < lowerRightCorner[i] && velocity.translation()(i) < 0)
    {
      return false;
    }
  }
  return true;
}

void BoundingBox::setTCPDistance(double value)
{
  rcmDistanceZ = value;
}
void BoundingBox::setTCP(Eigen::Affine3d TCP)
{
  TCP_old = TCP;
}

void BoundingBox::setRCM(Eigen::Vector3d remoteCenterOfMotion)
{
  RCM = remoteCenterOfMotion;
}

void BoundingBox::setBoundingBox(Eigen::Vector3d value)
{
  boundingBoxSize = value;
}
