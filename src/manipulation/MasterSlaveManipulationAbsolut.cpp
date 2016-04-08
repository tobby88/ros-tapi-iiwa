#include "masterslave/manipulation/MasterSlaveManipulationAbsolut.h"

MasterSlaveManipulationAbsolute::MasterSlaveManipulationAbsolute(ros::NodeHandle &nh): nh_(nh), markerSub(nh_,"hand/ar_pose_marker",1), markerSubRef(nh_,"reference/ar_pose_marker",1)
{
    message_filters::TimeSynchronizer<ar_track_alvar_msgs::AlvarMarkers,ar_track_alvar_msgs::AlvarMarkers> sync(markerSub,markerSubRef, 10);
    sync.registerCallback(boost::bind(&MasterSlaveManipulationAbsolute::markerCallback,this,_1,_2));
    masterSlaveServer = nh_.advertiseService("/Manipulation",&MasterSlaveManipulationAbsolute::masterSlaveCallback,this);
    cycleTimeSub = nh_.subscribe("/cycleTime",1,&MasterSlaveManipulationAbsolute::cycleTimeCallback,this);
    poseAct = Eigen::Affine3d::Identity();
    poseOld = Eigen::Affine3d::Identity();
    difference = Eigen::Affine3d::Identity();
    ros::spin();
    lastTime = ros::Time::now().toSec();
}

void MasterSlaveManipulationAbsolute::markerCallback(const ar_track_alvar_msgs::AlvarMarkersConstPtr &handMarker, const ar_track_alvar_msgs::AlvarMarkersConstPtr &referenceMarker)
{
    frameTime = ros::Time::now().toSec() - lastTime;
    lastTime = ros::Time::now().toSec();
    Eigen::Affine3d referencePose = Eigen::Affine3d::Identity();
    difference = Eigen::Affine3d::Identity();
    Eigen::Quaterniond differenceRot;
    for(int i=0; i<referenceMarker->markers.size();i++)
    {
        if(referenceMarker->markers[i].id == 4)
        {
            tf::poseMsgToEigen(referenceMarker->markers[i].pose.pose,referencePose);
            break;
        }
    }
    for(int i=0; i<handMarker->markers.size();i++)
    {
        if(handMarker->markers[i].id == 0)
        {
            poseOld = poseAct;
            tf::poseMsgToEigen(handMarker->markers[i].pose.pose,poseAct);
            if(initialRun)
            {
                poseOld = poseAct;
                initialRun = false;
                return;
            }
            //difference.translation() = referencePose.rotation()*(poseOld.inverse()*poseAct).translation()*cycleTime/frameTime;
            differenceRot = Eigen::Quaterniond::Identity().slerp(cycleTime/frameTime,Eigen::Quaterniond((poseOld.inverse()*poseAct).rotation()));
            difference.rotate(differenceRot.toRotationMatrix());
            break;
        }
    }
}

bool MasterSlaveManipulationAbsolute::masterSlaveCallback(masterslave::Manipulation::Request &req, masterslave::Manipulation::Response &resp)
{
    Eigen::Affine3d T_0_EE_old;
    Eigen::Affine3d T_0_EE_new = Eigen::Affine3d::Identity();

    tf::poseMsgToEigen(req.T_0_EE_old,T_0_EE_old);
    T_0_EE_new.translate((T_0_EE_old*difference).translation());
    T_0_EE_new.rotate(difference.rotation());
    T_0_EE_new.rotate(T_0_EE_old.rotation());
    tf::poseEigenToMsg(T_0_EE_new,resp.T_0_EE_new);
    ROS_DEBUG_STREAM(difference.matrix());
    return true;
}

void MasterSlaveManipulationAbsolute::cycleTimeCallback(const std_msgs::Float64ConstPtr &val)
{
    cycleTime = val->data;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "MasterSlaveManipulationAbsolute");
    ros::NodeHandle MasterSlaveManipulationNH;
    MasterSlaveManipulationAbsolute* manipulation = new MasterSlaveManipulationAbsolute(MasterSlaveManipulationNH);

}
