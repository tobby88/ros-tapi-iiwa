#include "masterslave/manipulation/VisualServoing.h"

VisualServoing::VisualServoing(ros::NodeHandle &nh): nh_(nh)
{
    markerJointRotation = Eigen::Affine3d::Identity();
    differenceTransform = Eigen::Affine3d::Identity();
    differenceTransformOld = Eigen::Affine3d::Identity();
    dynamic_reconfigure::Server<masterslave::VisualServoingConfig> server;
    dynamic_reconfigure::Server<masterslave::VisualServoingConfig>::CallbackType f;
    f = boost::bind(&VisualServoing::configurationCallback,this ,_1,_2);
    server.setCallback(f);
    getControlDevice();
    markerSub = nh_.subscribe("/visualServoing/ar_pose_marker",1,&VisualServoing::markerCallback,this);
    markerJointAngleSub = nh_.subscribe("/Q6P/joint_states",1,&VisualServoing::markerJointAngleCallback,this);
    visualServoingServer = nh_.advertiseService("/Manipulation",&VisualServoing::visualServoingCallback,this);

    Eigen::Vector3d anglesMarkerTool;
    anglesMarkerTool << -M_PI/2-MARKER_TO_TCP_ANGLE+markerJointAngle,0,M_PI;
    markerJointRotation = Eigen::Affine3d::Identity();
    markerJointRotation.rotate(QuaternionFromEuler(anglesMarkerTool,true));

    lastTime = ros::Time::now().toSec();
    lastMarkerTime = ros::Time::now().toNSec();
    ros::spin();
}

void VisualServoing::getControlDevice()
{
    std::stringstream device_sstream;
    XmlRpc::XmlRpcValue deviceList;

    // TODO: Laparoskop-Kinematik einbinden
    if(strcmp(nh_.getNamespace().c_str(),"/Joy")==0)
    {
       nh_.getParam("/API/Joy",deviceList);
    }
    else if(strcmp(nh_.getNamespace().c_str(),"/Spacenav")==0)
    {
        nh_.getParam("/API/Spacenav",deviceList);
    }
    else
    {
        ROS_ERROR("No ControlDevice found!");
        return;
    }
    //TODO: Auswahl des gewünschten Steuerungsgeräts
    for(XmlRpc::XmlRpcValue::iterator it = deviceList.begin(); it!=deviceList.end();it++)
    {
        device_sstream << it->first << "/Velocity";
        ROS_INFO_STREAM(device_sstream.str());
        velocitySub = nh_.subscribe(device_sstream.str().c_str(),10,&VisualServoing::velocityCallback,this);
        device_sstream.str(std::string());
    }
}

Eigen::Vector3d VisualServoing::calculateTranslationalPID()
{
    Eigen::Vector3d retValue = Eigen::Vector3d::Zero();
    retValue = pTrans*differenceTransform.translation() + dTrans*(differenceTransform.translation()-differenceTransformOld.translation())/markerCycleTime ;
    return retValue;
}

Eigen::Quaterniond VisualServoing::calculateRotationalPID()
{
    Eigen::Quaterniond initialRotation = Eigen::Quaterniond(initialTransform.rotation());
    Eigen::Quaterniond actualRotation = Eigen::Quaterniond(actualTransform.rotation());


    return actualRotation.slerp(pRot,initialRotation)*actualRotation.inverse();
}

void VisualServoing::velocityCallback(const geometry_msgs::TwistStampedConstPtr val)
{
    velocity = *val;

    if(!initialRun)
    {
        Eigen::Affine3d manipulatedInitialTransform = Eigen::Affine3d::Identity();

        manipulatedInitialTransform.translate(initialTransform.translation());

        manipulatedInitialTransform.translate(Eigen::Vector3d(velocity.twist.linear.x*markerCycleTime,velocity.twist.linear.y*markerCycleTime,velocity.twist.linear.z*markerCycleTime));
        manipulatedInitialTransform.rotate(QuaternionFromEuler(Eigen::Vector3d(velocity.twist.angular.x*markerCycleTime,velocity.twist.angular.y*markerCycleTime,velocity.twist.angular.z*markerCycleTime),true));
        manipulatedInitialTransform.rotate(initialTransform.rotation());
        initialTransform = manipulatedInitialTransform;
    }
}

bool VisualServoing::visualServoingCallback(masterslave::Manipulation::Request &req, masterslave::Manipulation::Response &resp)
{

    Eigen::Affine3d T_0_EE_old;
    Eigen::Affine3d T_0_EE_new = Eigen::Affine3d::Identity();
    cycleTime = ros::Time::now().toSec() - lastTime;
    lastTime = ros::Time::now().toSec();
    tf::poseMsgToEigen(req.T_0_EE_old,T_0_EE_old);
    if(markerFoundObject || markerFoundTCP || !initialRun || !initialRunVisualServoing)
    {
        Eigen::Quaterniond oldRotation = Eigen::Quaterniond(T_0_EE_old.rotation());
        Eigen::Vector3d feedForwardTrans = Eigen::Vector3d::Zero();
        feedForwardTrans << velocity.twist.linear.x, velocity.twist.linear.y, velocity.twist.linear.z;
        ROS_INFO_STREAM("PD-Trans" << calculateTranslationalPID());
        ROS_INFO_STREAM("Feedforward-Trans" << feedForwardTrans);
        T_0_EE_new.translate(T_0_EE_old.translation()-calculateTranslationalPID()*(cycleTime/markerCycleTime)+feedForwardTrans*markerCycleTime);

        Eigen::Vector3d eulerZYX;
        eulerZYX << velocity.twist.angular.x, velocity.twist.angular.y, velocity.twist.angular.z;
        Eigen::Quaterniond feedForwardRot = QuaternionFromEuler(eulerZYX*markerCycleTime,true);
        ROS_INFO_STREAM("PD-Rot" << calculateRotationalPID().toRotationMatrix());
        ROS_INFO_STREAM("Feedforward-Rot" << feedForwardRot.toRotationMatrix());
        T_0_EE_new.rotate(oldRotation.slerp(cycleTime/markerCycleTime,oldRotation*calculateRotationalPID())*feedForwardRot);
    }
    else
    {
        T_0_EE_new = T_0_EE_old;
    }
    tf::poseEigenToMsg(T_0_EE_new,resp.T_0_EE_new);
    return true;

}

void VisualServoing::markerCallback(const ar_track_alvar_msgs::AlvarMarkersConstPtr& markerPtr)
{
    markerFoundTCP=false;
    markerFoundObject=false;
    Eigen::Affine3d tcp;
    Eigen::Affine3d obj;
    differenceTransformOld = differenceTransform;
    differenceTransform = Eigen::Affine3d::Identity();
    actualTransform = Eigen::Affine3d::Identity();
    markerCycleTime = ros::Time::now().toSec() - lastMarkerTime;
    lastMarkerTime = ros::Time::now().toSec();
    for(int i=0; i< markerPtr->markers.size(); i++)
    {
        switch(markerPtr->markers[i].id)
        {
            case 10:
            tf::poseMsgToEigen(markerPtr->markers[i].pose.pose,tcp);
            if(initialRun)
            {
                tcpOld = tcp;
            }
            else
            {
                tcp = filterPose(tcp, tcpOld,MINIMAL_DISTANCE,MINIMAL_STEP_DISTANCE);
            }
            //Drehung des Markers in das TCP-System
            tcpOld = tcp;
            markerFoundTCP = true;
            ROS_INFO_STREAM("TCP" << tcp.matrix());
            break;

            case 11:
            tf::poseMsgToEigen(markerPtr->markers[i].pose.pose,obj);
            if(initialRun)
            {
                objOld = obj;
            }
            else
            {
                obj= filterPose(obj, objOld,MINIMAL_DISTANCE,MINIMAL_STEP_DISTANCE);
            }
            objOld = obj;
            markerFoundObject = true;
            ROS_INFO_STREAM("OBJ" << obj.matrix());
            break;
        }
    }

    if(!markerFoundObject || !markerFoundTCP)
    {
        ROS_ERROR_THROTTLE(1,"Marker unsichtbar!");
        return;
    }
    // Hier fehlt noch die Transformation ins TCP-System
    actualTransform.translate(obj.translation()-tcp.translation());
    actualTransform.rotate(tcp.rotation().inverse()*obj.rotation());
    if(initialRun)
    {
        initialTransform = actualTransform;
        initialRun = false;
    }
    // neue Toollage im alten Toolkoordinatensystem
    ROS_WARN_STREAM("actualTransform" << actualTransform.matrix());
    differenceTransform.translate(markerJointRotation.rotation()*(initialTransform.translation()-actualTransform.translation()));
    differenceTransform.rotate(markerJointRotation.rotation()*actualTransform.rotation().inverse()*initialTransform.rotation());
    ROS_WARN_STREAM("differenceTransform" << differenceTransform.matrix());

}

void VisualServoing::configurationCallback(masterslave::VisualServoingConfig &config, uint32_t level)
{
    pRot = config.PRot;
    dRot = config.DRot;

    pTrans = config.PTrans;
    dTrans = config.DTrans;

    if(config.ResetInit != resetInitOld)
    {
        initialRun = true;
        resetInitOld == config.ResetInit;
    }
}



void VisualServoing::markerJointAngleCallback(const sensor_msgs::JointStateConstPtr &state)
{
    markerJointAngle = state->position[0];
    Eigen::Vector3d anglesMarkerTool;
    anglesMarkerTool << -M_PI/2-MARKER_TO_TCP_ANGLE+markerJointAngle,0,M_PI;
    markerJointRotation = Eigen::Affine3d::Identity();
    markerJointRotation.rotate(QuaternionFromEuler(anglesMarkerTool,true));


}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"VisualServoing");
    ros::NodeHandle VisualServoingNH(argv[1]);
    VisualServoing* visualServoing = new VisualServoing(VisualServoingNH);
}
