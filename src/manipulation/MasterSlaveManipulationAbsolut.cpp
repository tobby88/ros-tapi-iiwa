#include "masterslave/manipulation/MasterSlaveManipulationAbsolut.h"

MasterSlaveManipulationAbsolute::MasterSlaveManipulationAbsolute(ros::NodeHandle &nh): nh_(nh), markerSub(nh_,"hand/ar_pose_marker",1), markerSubRef(nh_,"reference/ar_pose_marker",1)
{
    // Nachrichtensynchronisation
    message_filters::TimeSynchronizer<ar_track_alvar_msgs::AlvarMarkers,ar_track_alvar_msgs::AlvarMarkers> sync(markerSub,markerSubRef, 10);
    sync.registerCallback(boost::bind(&MasterSlaveManipulationAbsolute::markerCallback,this,_1,_2));
    masterSlaveServer = nh_.advertiseService("/Manipulation",&MasterSlaveManipulationAbsolute::masterSlaveCallback,this);
    cycleTimeSub = nh_.subscribe("/cycleTime",1,&MasterSlaveManipulationAbsolute::cycleTimeCallback,this);
    poseAct = Eigen::Affine3d::Identity();
    poseOld = Eigen::Affine3d::Identity();
    difference = Eigen::Affine3d::Identity();
    lastTime = ros::Time::now().toSec();
    ros::spin();
}

void MasterSlaveManipulationAbsolute::markerCallback(const ar_track_alvar_msgs::AlvarMarkersConstPtr &handMarker, const ar_track_alvar_msgs::AlvarMarkersConstPtr &referenceMarker)
{
    frameTime = ros::Time::now().toSec() - lastTime;
    lastTime = ros::Time::now().toSec();
    Eigen::Affine3d referencePose = Eigen::Affine3d::Identity();
    difference = Eigen::Affine3d::Identity();

    for(int i=0; i<referenceMarker->markers.size();i++)
    {
        referenceMarkerFound = false;
        if(referenceMarker->markers[i].id == 4)
        {
            tf::poseMsgToEigen(referenceMarker->markers[i].pose.pose,referencePose);
            referenceMarkerFound = true;
            break;
        }
    }
    for(int i=0; i<handMarker->markers.size();i++)
    {
        handMarkerFound = false;
        if(handMarker->markers[i].id == 0)
        {
            handMarkerFound = true;
            poseOld = poseAct;
            tf::poseMsgToEigen(handMarker->markers[i].pose.pose,poseAct);
            ROS_INFO_STREAM("poseAct: \n" << poseAct.matrix());
            if(initialRun)
            {
                poseOld = poseAct;
                initialRun = false;
                return;
            }
            ROS_WARN_STREAM("poseOld: \n" << poseOld.matrix());
            difference.translate((poseOld.inverse()*poseAct).translation()*cycleTime/frameTime);
            ROS_INFO_STREAM("difference: \n" << difference.translation());
            slerpParameter = 0;
            break;
        }
    }
}

bool MasterSlaveManipulationAbsolute::masterSlaveCallback(masterslave::Manipulation::Request &req, masterslave::Manipulation::Response &resp)
{
    //Inkrement, da ceil(frameTime/cycleTime) Zyklen gebraucht werden, um die Interpolation durchzuführen
    slerpParameter += cycleTime/frameTime;
    /*
     * slerpParameter hat einen Wertebereich von 0 bis 1
     * für die Rotationsinterpolation
     */
    if(slerpParameter >= 1) slerpParameter =0;


    Eigen::Affine3d T_0_EE_old = Eigen::Affine3d::Identity();
    Eigen::Affine3d T_0_EE_new = Eigen::Affine3d::Identity();

    Eigen::Quaterniond differenceRot = Eigen::Quaterniond((poseOld.inverse()*poseAct).rotation());
    ROS_DEBUG_STREAM("rotation Difference: \n" << differenceRot.toRotationMatrix());
    Eigen::Quaterniond oldRotation = Eigen::Quaterniond(T_0_EE_old.rotation());

    tf::poseMsgToEigen(req.T_0_EE_old,T_0_EE_old);
    ROS_DEBUG_STREAM(T_0_EE_old.matrix());
    T_0_EE_new.translate((T_0_EE_old*difference).translation());
    ROS_DEBUG_STREAM(T_0_EE_new.matrix());
    T_0_EE_new.rotate(oldRotation.slerp(slerpParameter,oldRotation*differenceRot));

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
