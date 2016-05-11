#include "masterslave/manipulation/VisualServoing.h"

VisualServoing::VisualServoing(ros::NodeHandle &nh): nh_(nh)
{
    differenceTransform = Eigen::Affine3d::Identity();
    differenceTransformOld = Eigen::Affine3d::Identity();
    dynamic_reconfigure::Server<masterslave::VisualServoingConfig> server;
    dynamic_reconfigure::Server<masterslave::VisualServoingConfig>::CallbackType f;
    f = boost::bind(&VisualServoing::configurationCallback,this ,_1,_2);
    server.setCallback(f);
    markerSub = nh_.subscribe("visualServoing/ar_pose_marker",1,&VisualServoing::markerCallback,this);
    visualServoingServer = nh_.advertiseService("/Manipulation",&VisualServoing::visualServoingCallback,this);
    ros::spin();
}

Eigen::Vector3d VisualServoing::calculateTranslationalPID()
{
    Eigen::Vector3d retValue = Eigen::Vector3d::Zero();

    retValue = pTrans*differenceTransform.translation() + dTrans*(differenceTransform.translation()-differenceTransformOld.translation())/cycleTime;
    ROS_INFO_STREAM(retValue);
    return retValue;
}

Eigen::Matrix3d VisualServoing::calculateRotationalPID()
{
    Eigen::Quaterniond temp;
    Eigen::Quaterniond acutalRotation(actualTransform.rotation());
    Eigen::Quaterniond inititialRotaion(initialTransform.rotation());
    temp = acutalRotation.slerp(pRot*cycleTime,inititialRotaion);
    ROS_INFO_STREAM(temp.toRotationMatrix());
    return temp.toRotationMatrix();
}

bool VisualServoing::visualServoingCallback(masterslave::Manipulation::Request &req, masterslave::Manipulation::Response &resp)
{
    Eigen::Affine3d T_0_EE_old;
    Eigen::Affine3d T_0_EE_new = Eigen::Affine3d::Identity();
    tf::poseMsgToEigen(req.T_0_EE_old,T_0_EE_old);
    T_0_EE_new.translate(T_0_EE_old.translation()-0.1*calculateTranslationalPID());
    T_0_EE_new.rotate(T_0_EE_old.rotation());
    ROS_WARN_STREAM(T_0_EE_old.matrix());
    ROS_WARN_STREAM(T_0_EE_new.matrix());
    tf::poseEigenToMsg(T_0_EE_new,resp.T_0_EE_new);
    return true;

}

void VisualServoing::markerCallback(const ar_track_alvar_msgs::AlvarMarkersConstPtr& markerPtr)
{
    bool markerFoundTCP=false;
    bool markerFoundObject=false;
    Eigen::Affine3d tcp;
    Eigen::Affine3d obj;
    differenceTransformOld = differenceTransform;
    differenceTransform = Eigen::Affine3d::Identity();
    for(int i=0; i< markerPtr->markers.size(); i++)
    {
        switch(markerPtr->markers[i].id)
        {
            case 0:
            tf::poseMsgToEigen(markerPtr->markers[i].pose.pose,tcp);
            markerFoundTCP = true;
            break;
            case 1:
            tf::poseMsgToEigen(markerPtr->markers[i].pose.pose,obj);
            markerFoundObject = true;
            break;
        }
    }

    if(!markerFoundObject || !markerFoundTCP)
    {
        ROS_ERROR("Marker unsichtbar!");
        return;
    }
        actualTransform = obj*tcp.inverse();
        if(initialRun)
        {
            initialTransform = actualTransform;
            initialRun = false;
        }
        differenceTransform = initialTransform.inverse()*actualTransform;
        ROS_WARN_STREAM(differenceTransform.matrix());

}

void VisualServoing::configurationCallback(masterslave::VisualServoingConfig &config, uint32_t level)
{
    pRot = config.PRot;
    dRot = config.DRot;

    pTrans = config.PTrans;
    dTrans = config.DTrans;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"VisualServoing");
    ros::NodeHandle VisualServoingNH;
    VisualServoing* visualServoing = new VisualServoing(VisualServoingNH);
}
