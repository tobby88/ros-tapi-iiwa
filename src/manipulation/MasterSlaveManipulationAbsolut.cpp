#include "masterslave/manipulation/MasterSlaveManipulationAbsolut.h"

MasterSlaveManipulationAbsolute::MasterSlaveManipulationAbsolute(ros::NodeHandle &nh): nh_(nh)
{
    markerSub = nh_.subscribe("/ar_pose_marker",1,&MasterSlaveManipulationAbsolute::markerCallback,this);
    masterSlaveServer = nh_.advertiseService("/Manipulation",&MasterSlaveManipulationAbsolute::masterSlaveCallback,this);
    cycleTimeSub = nh_.subscribe("/cycleTime",1,&MasterSlaveManipulationAbsolute::cycleTimeCallback,this);
    poseAct = Eigen::Affine3d::Identity();
    poseOld = Eigen::Affine3d::Identity();
    difference = Eigen::Affine3d::Identity();
    ros::spin();

}

void MasterSlaveManipulationAbsolute::markerCallback(const ar_track_alvar_msgs::AlvarMarkersConstPtr &controlMarker)
{
    difference = Eigen::Affine3d::Identity();
    Eigen::Quaterniond differenceRot;
    for(int i=0; i<controlMarker->markers.size();i++)
    {
        if(controlMarker->markers[i].id == 0)
        {
            tf::poseMsgToEigen(controlMarker->markers[i].pose.pose,poseAct);
            if(initialRun)
            {
                poseOld = poseAct;
                initialRun = false;
                return;
            }
            difference.translation() = (poseOld.inverse()*poseAct).translation()*30*cycleTime*0.01;
            differenceRot = Eigen::Quaterniond((poseOld.inverse()*poseAct).rotation());
            differenceRot = Eigen::Quaterniond::Identity().slerp(1/60,differenceRot);
            difference.rotate(differenceRot.toRotationMatrix());

        }
    }
}

bool MasterSlaveManipulationAbsolute::masterSlaveCallback(masterslave::Manipulation::Request &req, masterslave::Manipulation::Response &resp)
{
    Eigen::Affine3d T_0_EE_old;
    Eigen::Affine3d T_0_EE_new;

    tf::poseMsgToEigen(req.T_0_EE_old,T_0_EE_old);
    T_0_EE_new = T_0_EE_old*difference;
    tf::poseEigenToMsg(T_0_EE_new,resp.T_0_EE_new);
    poseOld = poseAct;
    ROS_WARN_STREAM(difference.matrix());
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
