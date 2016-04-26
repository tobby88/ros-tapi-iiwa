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
    //markerCallback
    lastFrameTime = ros::Time::now().toSec();
    //masterSlaveCallback
    lastMasterSlaveTime = ros::Time::now().toSec();
    ros::spin();
}

void MasterSlaveManipulationAbsolute::markerCallback(const ar_track_alvar_msgs::AlvarMarkersConstPtr &handMarker, const ar_track_alvar_msgs::AlvarMarkersConstPtr &referenceMarker)
{
    frameTime = ros::Time::now().toSec() - lastFrameTime;
    lastFrameTime = ros::Time::now().toSec();
    Eigen::Affine3d referencePose = Eigen::Affine3d::Identity();
    difference = Eigen::Affine3d::Identity();
    referenceMarkerFound = false;
    for(int i=0; i<referenceMarker->markers.size();i++)
    {

        if(referenceMarker->markers[i].id == 4)
        {
            tf::poseMsgToEigen(referenceMarker->markers[i].pose.pose,referencePose);
            referenceMarkerFound = true;
            break;
        }
    }
    handMarkerFound = false;
    for(int i=0; i<handMarker->markers.size();i++)
    {
        if(handMarker->markers[i].id == 0)
        {
            handMarkerFound = true;
            tf::poseMsgToEigen(handMarker->markers[i].pose.pose,poseAct);
            ROS_DEBUG_STREAM("poseAct: \n" << poseAct.matrix());
            if(initialRun)
            {
                poseOld = poseAct;
                initialRotationMarker = Eigen::Quaterniond(poseAct.rotation());
                initialRun = false;
                return;
            }
            ROS_DEBUG_STREAM("poseOld: \n" << poseOld.matrix());
            if((referencePose.inverse()*poseAct.translation()-referencePose.inverse()*poseOld.translation()).norm() >= 3e-03)
            {
                difference.translate((referencePose.inverse()*poseAct.translation()-referencePose.inverse()*poseOld.translation()));
            }

            ROS_DEBUG_STREAM("difference: \n" << difference.translation());
            slerpParameter = 0;
            break;
        }
    }
    if(!handMarkerFound && handMarker->markers.size()==0)
    {
        ROS_INFO("Marker unsichtbar!");
        //poseAct = poseOld;
    }
    markerCallbackCalled  = true;
}

bool MasterSlaveManipulationAbsolute::masterSlaveCallback(masterslave::Manipulation::Request &req, masterslave::Manipulation::Response &resp)
{

    Eigen::Affine3d T_0_EE_old = Eigen::Affine3d::Identity();

    tf::poseMsgToEigen(req.T_0_EE_old,T_0_EE_old);
    if(initialRunMasterSlave)
    {
        initialPoseRobot = T_0_EE_old.translation();
        initialRotationRobot = Eigen::Quaterniond(T_0_EE_old.rotation());
        initialRunMasterSlave = false;

    }
    oldRotation = Eigen::Quaterniond(T_0_EE_old.rotation());
    Eigen::Affine3d T_0_EE_new = Eigen::Affine3d::Identity();
    masterSlaveTime = ros::Time::now().toSec() - lastMasterSlaveTime;
    lastMasterSlaveTime = ros::Time::now().toSec();
    if(!markerCallbackCalled || !handMarkerFound || !referenceMarkerFound)
    {
        resp.T_0_EE_new = req.T_0_EE_old;
        return true;
    }

    //Inkrement, da ceil(frameTime/masterSlaveTime) Zyklen gebraucht werden, um die Interpolation durchzuführen
    slerpParameter += masterSlaveTime/frameTime;

    /*
     * slerpParameter hat einen Wertebereich von 0 bis 1
     * für die Rotationsinterpolation
     */
    if(slerpParameter >= 1) slerpParameter =0;



    Eigen::Quaterniond differenceRot = Eigen::Quaterniond(poseAct.rotation());
    ROS_DEBUG_STREAM("rotation Difference: \n" << differenceRot.toRotationMatrix());


    //<ROS_INFO_STREAM(T_0_EE_old.matrix());

    T_0_EE_new.translate(T_0_EE_old.translation());
    T_0_EE_new.translate((initialPoseRobot+difference.translation()-T_0_EE_old.translation())*masterSlaveTime/frameTime);


    //T_0_EE_new.rotate(oldRotation.slerp(slerpParameter,oldRotation*differenceRot));

    Eigen::Quaterniond newRotation = initialRotationRobot*initialRotationMarker.inverse()*differenceRot;
    T_0_EE_new.rotate(oldRotation.slerp(slerpParameter,newRotation));

    ROS_DEBUG_STREAM("Rotation T_0_EE_new: \n" << T_0_EE_new.rotation());
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
