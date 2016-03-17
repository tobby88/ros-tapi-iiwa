#ifndef BOUNDINGBOX_H
#define BOUNDINGBOX_H

#include <Eigen/Dense>
// just for vrep
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
class BoundingBox
{
public:
    BoundingBox(ros::NodeHandle& nh, Eigen::Affine3d TCP, Eigen::Vector3d RemoteCenterOfMotion, Eigen::Vector3d boundingBoxSizeVector, double rcmDistance);
    void setTCPDistance(double value);
    void setBoundingBox(Eigen::Vector3d value);
    void setTCP(Eigen::Affine3d TCP);
    void setRCM(Eigen::Vector3d remoteCenterOfMotion);
    bool checkBoundingBoxTCP(Eigen::Affine3d);
    int getAllThingsSet(){ return allThingsSet;}
private:
    ros::NodeHandle nh_;
    ros::Publisher centerPub;
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
