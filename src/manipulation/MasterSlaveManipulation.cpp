#include "masterslave/manipulation/MasterSlaveManipulation.h"

MasterSlaveManipulation::MasterSlaveManipulation(ros::NodeHandle &nh): nh_(nh)
{
    getControlDevice();
    cycleTimeSub = nh_.subscribe("/cycleTime",1,&MasterSlaveManipulation::cycleTimeCallback,this);
    masterSlaveServer = nh_.advertiseService("/Manipulation",&MasterSlaveManipulation::masterSlaveCallback,this);
    /* MasterSlaveMode
     * 1: MasterSlaveRelative
     * 2: MasterSlaveAbsolute
     */
    if(nh_.hasParam("/masterSlaveMode"))
    {
        nh_.deleteParam("/masterSlaveMode");
    }
    nh_.setParam("/masterSlaveMode",1);

    ros::spin();
}

void MasterSlaveManipulation::getControlDevice()
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
        velocitySub = nh_.subscribe(device_sstream.str().c_str(),10,&MasterSlaveManipulation::velocityCallback,this);
        device_sstream.str(std::string());
    }
}

void MasterSlaveManipulation::velocityCallback(const geometry_msgs::TwistStampedConstPtr &val)
{
    velocity = *val;
}

void MasterSlaveManipulation::cycleTimeCallback(const std_msgs::Float64ConstPtr &val)
{
    cycleTime = val->data;
}

bool MasterSlaveManipulation::masterSlaveCallback(masterslave::Manipulation::Request &req, masterslave::Manipulation::Response &resp)
{
    Eigen::Affine3d TCPold;
    Eigen::Affine3d TCPnew;
    tf::poseMsgToEigen(req.T_0_EE_old,TCPold);

    TCPnew.setIdentity();
    TCPnew.translate(TCPold.translation());

    TCPnew.translate(Eigen::Vector3d(velocity.twist.linear.x*cycleTime,velocity.twist.linear.y*cycleTime,0));

    TCPnew.translate(Eigen::Vector3d(0,0,velocity.twist.linear.z*cycleTime));


    TCPnew.rotate(QuaternionFromEuler(Eigen::Vector3d(velocity.twist.angular.x*cycleTime,velocity.twist.angular.y*cycleTime,velocity.twist.angular.z*cycleTime),true));
    TCPnew.rotate(TCPold.rotation());

    tf::poseEigenToMsg(TCPnew,resp.T_0_EE_new);
    return true;

}


Eigen::Quaternion<double> MasterSlaveManipulation::QuaternionFromEuler(const Eigen::Vector3d &eulerXYZ, bool ZYX=true)
{
    Eigen::Quaternion<double> quat;
    quat.Identity();
    Eigen::AngleAxisd zAngle(eulerXYZ[2], Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yAngle(eulerXYZ[1], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd xAngle(eulerXYZ[0], Eigen::Vector3d::UnitX());
    if(ZYX)
        quat = zAngle * yAngle * xAngle;
    else
        quat = xAngle * yAngle * zAngle;

    return quat;
}



int main(int argc, char** argv)
{
    ros::init(argc,argv,"MasterSlaveManipulation");
    ros::NodeHandle MasterSlaveManipulationNH(argv[1]);
    MasterSlaveManipulation* masterSlave = new MasterSlaveManipulation(MasterSlaveManipulationNH);
    return 0;
}
