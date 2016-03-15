#include "masterslave/kinematic/boundingbox.h"

BoundingBox::BoundingBox(Eigen::Affine3d TCP, Eigen::Vector3d RemoteCenterOfMotion, Eigen::Vector3d boundingBoxSizeVector, double rcmDistance)
{
    TCP_old = TCP;
    RCM = RemoteCenterOfMotion;
    boundingBoxSize = boundingBoxSizeVector;
    rcmDistanceZ = rcmDistance;
    upperLeftCorner = RCM + Eigen::Vector3d(boundingBoxSize(0)/2,boundingBoxSize(1)/2,-rcmDistanceZ);
    lowerRightCorner = upperLeftCorner - boundingBoxSize;
    centerOfBoundingBox = RCM - Eigen::Vector3d(0,0,rcmDistanceZ+boundingBoxSize(2));
}

bool BoundingBox::checkBoundingBoxTCP(Eigen::Affine3d TCP)
{
    Eigen::Affine3d velocity = TCP_old.inverse()*TCP;
    for(int i=0;i<3;i++)
    {
        if(TCP.translation()(i)>upperLeftCorner[i])
        {
            return false;
        }
        else if(TCP.translation()(i)<lowerRightCorner[i])
        {
            return false;
        }
    }
    return true;
}

void BoundingBox::setTCPDistance(double value)
{
    rcmDistanceZ = value;
    allThingsSet += 1;
}
void BoundingBox::setTCP(Eigen::Affine3d TCP)
{
    TCP_old = TCP;
    allThingsSet += 1 << 1;
}

void BoundingBox::setRCM(Eigen::Vector3d remoteCenterOfMotion)
{
    RCM = remoteCenterOfMotion;
    allThingsSet += 1 << 2;
}

void BoundingBox::setBoundingBox(Eigen::Vector3d value)
{
    boundingBoxSize = value;
    allThingsSet += 1 << 3;
}


