#include "masterslave/manipulation/MasterSlaveManipulationAbsolut.h"

MasterSlaveManipulationAbsolute::MasterSlaveManipulationAbsolute(ros::NodeHandle &nh): nh_(nh), markerSub(nh_,"hand/ar_pose_marker",1), markerSubRef(nh_,"reference/ar_pose_marker",1)
{
    dynamic_reconfigure::Server<masterslave::MasterSlaveManipulationAbsoluteConfig> server;
    dynamic_reconfigure::Server<masterslave::MasterSlaveManipulationAbsoluteConfig>::CallbackType f;
    f = boost::bind(&MasterSlaveManipulationAbsolute::configurationCallback,this ,_1,_2);
    server.setCallback(f);

    // Nachrichtensynchronisation
    message_filters::TimeSynchronizer<ar_track_alvar_msgs::AlvarMarkers,ar_track_alvar_msgs::AlvarMarkers> sync(markerSub,markerSubRef, 10);
    sync.registerCallback(boost::bind(&MasterSlaveManipulationAbsolute::markerCallback,this,_1,_2));

    masterSlaveServer = nh_.advertiseService("/Manipulation",&MasterSlaveManipulationAbsolute::masterSlaveCallback,this);
    cycleTimeSub = nh_.subscribe("/cycleTime",1,&MasterSlaveManipulationAbsolute::cycleTimeCallback,this);


    poseAct = Eigen::Affine3d::Identity();
    poseOld = Eigen::Affine3d::Identity();
    difference = Eigen::Vector3d::Zero();
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
    difference = Eigen::Vector3d::Zero();
    referenceMarkerFound = false;

    // Suche des Referenzmarkers
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

    // Suche der Steuerungsmarker
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
            // Überwachung
            if((referencePose.inverse()*poseAct.translation()-referencePose.inverse()*poseOld.translation()).norm() >= 3e-03)
            {
                difference = transMotionScaling*((referencePose.inverse()*poseAct.translation()-referencePose.inverse()*poseOld.translation()));
            }

            ROS_DEBUG_STREAM("difference: \n" << difference);
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
    slerpParameter += std::ceil(masterSlaveTime/frameTime*10 + 0.5)/10;

    /*
     * slerpParameter hat einen Wertebereich von 0 bis 1
     * für die Rotationsinterpolation
     */
    if(slerpParameter >= 1) slerpParameter =0;



    Eigen::Quaterniond differenceRot = Eigen::Quaterniond(poseAct.rotation());
    ROS_DEBUG_STREAM("rotation Difference: \n" << differenceRot.toRotationMatrix());

    T_0_EE_new.translate(T_0_EE_old.translation());
    Eigen::Vector3d newTranslation = (initialPoseRobot+difference-T_0_EE_old.translation())*slerpParameter;

    // Geschwindigkeitsüberwachung
    if(newTranslation.norm()>0.01*slerpParameter)
    {
        newTranslation /=(newTranslation.norm()/(0.01*slerpParameter));
    }
    T_0_EE_new.translate(newTranslation);


    Eigen::Quaterniond newRotation = initialRotationRobot*initialRotationMarker.inverse()*differenceRot;


    Eigen::AngleAxisd differenceAngle = Eigen::AngleAxisd(newRotation*initialRotationRobot.inverse());

    // Abfangen, ob der globale Kippwinkel kleiner als 90° ist
    if(std::abs(differenceAngle.angle())<M_PI/2)
    {
        T_0_EE_new.rotate(oldRotation.slerp(slerpParameter,newRotation));
    }
    else
    {
        // Wenn nicht, dann wird die Rotation konstant auf dem alten Wert gehalten
        T_0_EE_new.rotate(oldRotation);
    }
    ROS_DEBUG_STREAM("Rotation T_0_EE_new: \n" << T_0_EE_new.rotation());
    tf::poseEigenToMsg(T_0_EE_new,resp.T_0_EE_new);
    ROS_DEBUG_STREAM(difference);
    return true;

}

void MasterSlaveManipulationAbsolute::cycleTimeCallback(const std_msgs::Float64ConstPtr &val)
{
    cycleTime = val->data;
}

void MasterSlaveManipulationAbsolute::configurationCallback(masterslave::MasterSlaveManipulationAbsoluteConfig &config, uint32_t level)
{
    transMotionScaling = config.motionScaling;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "MasterSlaveManipulationAbsolute");
    ros::NodeHandle MasterSlaveManipulationNH;
    MasterSlaveManipulationAbsolute* manipulation = new MasterSlaveManipulationAbsolute(MasterSlaveManipulationNH);

}
