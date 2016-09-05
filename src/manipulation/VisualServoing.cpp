#include "masterslave/manipulation/VisualServoing.h"

VisualServoing::VisualServoing(ros::NodeHandle &nh) : nh_(nh)
{
  markerJointRotation = Eigen::Affine3d::Identity();
  differenceTransform = Eigen::Affine3d::Identity();
  differenceTransformOld = Eigen::Affine3d::Identity();

  dynamic_reconfigure::Server<masterslave::VisualServoingConfig>::CallbackType f;
  f = boost::bind(&VisualServoing::configurationCallback, this, _1, _2);
  server.setCallback(f);
  getControlDevice();
  markerSub = nh_.subscribe("/visualServoing/ar_pose_marker", 1, &VisualServoing::markerCallback, this);

  visualServoingServer = nh_.advertiseService("/Manipulation", &VisualServoing::visualServoingCallback, this);

  message_filters::Subscriber<sensor_msgs::JointState> Q6pSub(nh_, "/Q6P/joint_states", 1);
  message_filters::Subscriber<sensor_msgs::JointState> Q6nSub(nh_, "/Q6N/joint_states", 1);
  message_filters::TimeSynchronizer<sensor_msgs::JointState, sensor_msgs::JointState> sync(Q6pSub, Q6nSub, 10);
  sync.registerCallback(boost::bind(&VisualServoing::markerJointAngleCallback, this, _1, _2));

  differenceTransformPub = nh_.advertise<geometry_msgs::Pose>("/controlError", 1);

  Eigen::Vector3d anglesMarkerTool;
  anglesMarkerTool << M_PI / 2 - MARKER_TO_TCP_ANGLE - markerJointAngle, 0, M_PI;
  markerJointRotation = Eigen::Affine3d::Identity();
  markerJointRotation.rotate(QuaternionFromEuler(anglesMarkerTool, true));
  // markerJointRotation.rotate(Eigen::AngleAxisd(M_PI,Eigen::Vector3d::UnitZ()));

  /* MasterSlaveMode
   * 1: MasterSlaveRelative
   * 2: MasterSlaveAbsolute
   */
  if (nh_.hasParam("/masterSlaveMode"))
  {
    nh_.deleteParam("/masterSlaveMode");
  }
  nh_.setParam("/masterSlaveMode", 1);

  lastTime = ros::Time::now().toSec();
  lastMarkerTime = ros::Time::now().toNSec();
  ros::spin();
}

void VisualServoing::getControlDevice()
{
  std::stringstream device_sstream;
  XmlRpc::XmlRpcValue deviceList;

  // TODO: Laparoskop-Kinematik einbinden
  if (strcmp(nh_.getNamespace().c_str(), "/Joy") == 0)
  {
    nh_.getParam("/API/Joy", deviceList);
  }
  else if (strcmp(nh_.getNamespace().c_str(), "/Spacenav") == 0)
  {
    nh_.getParam("/API/Spacenav", deviceList);
  }
  else
  {
    ROS_ERROR("No ControlDevice found!");
    return;
  }
  // TODO: Auswahl des gewünschten Steuerungsgeräts
  for (XmlRpc::XmlRpcValue::iterator it = deviceList.begin(); it != deviceList.end(); it++)
  {
    device_sstream << it->first << "/Velocity";
    ROS_INFO_STREAM(device_sstream.str());
    velocitySub = nh_.subscribe(device_sstream.str().c_str(), 10, &VisualServoing::velocityCallback, this);
    device_sstream.str(std::string());
  }
}

Eigen::Vector3d VisualServoing::calculateTranslationalPID()
{
  Eigen::Vector3d retValue = Eigen::Vector3d::Zero();
  retValue = pTrans * differenceTransform.translation() +
             dTrans * (differenceTransform.translation() - differenceTransformOld.translation()) / markerCycleTime;
  return retValue;
}

Eigen::Quaterniond VisualServoing::calculateRotationalPID()
{
  return Eigen::Quaterniond::Identity().slerp(pRot, Eigen::Quaterniond(differenceTransform.rotation()));
}

void VisualServoing::velocityCallback(const geometry_msgs::TwistStampedConstPtr val)
{
  velocity = *val;

  if (!initialRun)
  {
    manipulateTransform(
        Eigen::Vector3d(velocity.twist.linear.x * markerCycleTime, velocity.twist.linear.y * markerCycleTime,
                        velocity.twist.linear.z * markerCycleTime),
        Eigen::Vector3d(velocity.twist.angular.x * markerCycleTime, velocity.twist.angular.y * markerCycleTime,
                        velocity.twist.angular.z * markerCycleTime));
  }
}

bool VisualServoing::visualServoingCallback(masterslave::Manipulation::Request &req,
                                            masterslave::Manipulation::Response &resp)
{
  Eigen::Affine3d T_0_EE_old;
  Eigen::Affine3d T_0_EE_new = Eigen::Affine3d::Identity();
  cycleTime = ros::Time::now().toSec() - lastTime;
  lastTime = ros::Time::now().toSec();
  tf::poseMsgToEigen(req.T_0_EE_old, T_0_EE_old);

  // Falls die  Marker nicht gefunden wurden
  if (!markerFoundObject || !markerFoundTCP)
  {
    T_0_EE_new = T_0_EE_old;
    tf::poseEigenToMsg(T_0_EE_new, resp.T_0_EE_new);
    return true;
  }

  Eigen::Vector3d feedforwardTranslational =
      Eigen::Vector3d(velocity.twist.linear.x, velocity.twist.linear.y, velocity.twist.linear.z);
  T_0_EE_new.translate(T_0_EE_old.translation());
  T_0_EE_new.translate(cycleTime * (calculateTranslationalPID() + feedforwardTranslational));

  T_0_EE_new.rotate(T_0_EE_old.rotation());
  T_0_EE_new.rotate(Eigen::Quaterniond::Identity().slerp(cycleTime, calculateRotationalPID()));
  tf::poseEigenToMsg(T_0_EE_new, resp.T_0_EE_new);
  return true;
}

void VisualServoing::markerCallback(const ar_track_alvar_msgs::AlvarMarkersConstPtr &markerPtr)
{
  markerFoundTCP = false;
  markerFoundObject = false;
  Eigen::Affine3d tcp;
  Eigen::Affine3d obj;
  differenceTransform = Eigen::Affine3d::Identity();
  actualTransform = Eigen::Affine3d::Identity();
  markerCycleTime = ros::Time::now().toSec() - lastMarkerTime;
  lastMarkerTime = ros::Time::now().toSec();
  for (int i = 0; i < markerPtr->markers.size(); i++)
  {
    switch (markerPtr->markers[i].id)
    {
      case 10:

        tf::poseMsgToEigen(markerPtr->markers[i].pose.pose, tcp);
        markerFoundTCP = true;

        break;

      case 11:

        tf::poseMsgToEigen(markerPtr->markers[i].pose.pose, obj);
        markerFoundObject = true;

        break;
    }
  }

  if (!markerFoundObject || !markerFoundTCP)
  {
    ROS_ERROR_THROTTLE(1, "Marker unsichtbar!");
    return;
  }
  if (!initialRun)
  {
    tcp = checkAngle(tcp, tcpOld);
    obj = checkAngle(obj, objOld);
  }
  tcpOld = tcp;
  objOld = obj;

  // Hier fehlt noch die Transformation ins TCP-System (NICHT MEHR!)
  actualTransform = tcp.inverse() * obj * markerJointRotation;

  if (initialRun)
  {
    initialTransform = actualTransform;  // Delta X_Soll

    initialRun = false;
  }
  else
  {
  }

  // neue Toollage im alten Toolkoordinatensystem
  // differenceTransform =
  // (markerJointRotation.inverse()*actualTransform.inverse()*initialTransform*markerJointRotation);
  differenceTransform = actualTransform.inverse() * initialTransform;

  geometry_msgs::Pose differencePose;
  tf::poseEigenToMsg(differenceTransform, differencePose);
  differenceTransformPub.publish(differencePose);

  return;
}

void VisualServoing::configurationCallback(masterslave::VisualServoingConfig &config, uint32_t level)
{
  switch (level)
  {
    case 0:
      pRot = config.PRot;
      pTrans = config.PTrans;
      dTrans = config.DTrans;
      ROS_INFO_STREAM("KTrans: " << pTrans << " DTrans: " << dTrans << " KRot: " << pRot);
      break;
    case 1:
      Eigen::Vector3d translation = Eigen::Vector3d((double)config.XOffset * MM_TO_M, (double)config.YOffset * MM_TO_M,
                                                    (double)config.ZOffset * MM_TO_M);
      Eigen::Vector3d orientation =
          Eigen::Vector3d(config.AOffset * DEG_TO_RAD, config.BOffset * DEG_TO_RAD, config.COffset * DEG_TO_RAD);
      manipulateTransform(translation, orientation);
      config.XOffset = 0;
      config.YOffset = 0;
      config.ZOffset = 0;
      config.AOffset = 0;
      config.BOffset = 0;
      config.COffset = 0;
      server.updateConfig(config);
      break;
  }

  if (config.ResetInit != resetInitOld)
  {
    initialRun = true;
    resetInitOld == config.ResetInit;
  }
}

void VisualServoing::markerJointAngleCallback(const sensor_msgs::JointStateConstPtr &Q6Pstate,
                                              const sensor_msgs::JointStateConstPtr &Q6Nstate)
{
  markerJointAngle = (Q6Pstate->position[0] - Q6Nstate->position[0]) / 2;
  Eigen::Vector3d anglesMarkerTool;
  anglesMarkerTool << -M_PI / 2 - MARKER_TO_TCP_ANGLE + markerJointAngle, 0, M_PI;
  markerJointRotation = Eigen::Affine3d::Identity();
  markerJointRotation.rotate(QuaternionFromEuler(anglesMarkerTool, true));
}

void VisualServoing::manipulateTransform(Eigen::Vector3d translation, Eigen::Vector3d orientation)
{
  Eigen::Affine3d manipulatedInitialTransform = Eigen::Affine3d::Identity();

  manipulatedInitialTransform.translate(translation);
  manipulatedInitialTransform.translate(initialTransform.translation());
  manipulatedInitialTransform.rotate(QuaternionFromEuler(orientation, true));
  manipulatedInitialTransform.rotate(initialTransform.rotation());

  initialTransform = manipulatedInitialTransform;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "VisualServoing");
  ros::NodeHandle VisualServoingNH(argv[1]);
  VisualServoing *visualServoing = new VisualServoing(VisualServoingNH);
}
