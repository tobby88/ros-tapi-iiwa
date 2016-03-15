#ifndef BOUNDINGBOX_H
#define BOUNDINGBOX_H

#include <Eigen/Dense>

class BoundingBox
{
public:
    BoundingBox(Eigen::Affine3d TCP, Eigen::Vector3d RemoteCenterOfMotion, Eigen::Vector3d boundingBoxSizeVector, double rcmDistance);
    void setTCPDistance(double value);
    void setBoundingBox(Eigen::Vector3d value);
    void setTCP(Eigen::Affine3d TCP);
    void setRCM(Eigen::Vector3d remoteCenterOfMotion);
    bool checkBoundingBoxTCP(Eigen::Affine3d);
    int getAllThingsSet(){ return allThingsSet;}
private:
    Eigen::Vector3d RCM;
    Eigen::Affine3d TCP_old;
    double rcmDistanceZ;
    Eigen::Vector3d boundingBoxSize;
    Eigen::Vector3d upperLeftCorner;
    Eigen::Vector3d lowerRightCorner;
    Eigen::Vector3d centerOfBoundingBox;
    int allThingsSet{0};
};

#endif // BOUNDINGBOX_H
