#include "masterslave/manipulation/MasterSlaveManipulationAbsolut.h"

MasterSlaveManipulationAbsolute::MasterSlaveManipulationAbsolute(ros::NodeHandle &nh) : nh_(nh)
{
  dynamic_reconfigure::Server<masterslave::MasterSlaveManipulationAbsoluteConfig> server;
  dynamic_reconfigure::Server<masterslave::MasterSlaveManipulationAbsoluteConfig>::CallbackType f;
  f = boost::bind(&MasterSlaveManipulationAbsolute::configurationCallback, this, _1, _2);
  server.setCallback(f);

  // Nachrichtensynchronisation
  markerSub = nh_.subscribe("/hand/ar_pose_marker", 1, &MasterSlaveManipulationAbsolute::markerCallback, this);

  pliersDistancePub = nh_.advertise<std_msgs::Float64>("/pliersDistance", 1);

  masterSlaveServer =
      nh_.advertiseService("/Manipulation", &MasterSlaveManipulationAbsolute::masterSlaveCallback, this);
  cycleTimeSub = nh_.subscribe("/cycleTime", 1, &MasterSlaveManipulationAbsolute::cycleTimeCallback, this);

  poseAct = Eigen::Affine3d::Identity();
  poseThumb = Eigen::Affine3d::Identity();
  poseIndexFinger = Eigen::Affine3d::Identity();
  initialPoseMarker = Eigen::Affine3d::Identity();

  difference = Eigen::Vector3d::Zero();
  // markerCallback
  lastFrameTime = ros::Time::now().toSec();
  // masterSlaveCallback
  lastMasterSlaveTime = ros::Time::now().toSec();

  if (nh_.hasParam("/masterSlaveMode"))
  {
    nh_.deleteParam("/masterSlaveMode");
  }
  nh_.setParam("/masterSlaveMode", 2);

  ros::spin();
}

void MasterSlaveManipulationAbsolute::markerCallback(const ar_track_alvar_msgs::AlvarMarkersConstPtr &handMarker)
{
  frameTime = ros::Time::now().toSec() - lastFrameTime;
  lastFrameTime = ros::Time::now().toSec();
  poseAct = Eigen::Affine3d::Identity();
  difference = Eigen::Vector3d::Zero();

  thumbMarkerFound = false;
  indexFingerMarkerFound = false;

  // Suche der Steuerungsmarker
  for (int i = 0; i < handMarker->markers.size(); i++)
  {
    switch (handMarker->markers[i].id)
    {
      case 0:
        thumbMarkerFound = true;
        poseThumbOld = poseThumb;
        tf::poseMsgToEigen(handMarker->markers[i].pose.pose, poseThumb);
        poseThumb = filterPose(poseThumb, poseThumbOld, MINIMAL_DISTANCE, MINIMAL_STEP_DISTANCE);
        break;
      case 5:
        indexFingerMarkerFound = true;
        poseIndexFingerOld = poseIndexFinger;
        tf::poseMsgToEigen(handMarker->markers[i].pose.pose, poseIndexFinger);
        poseIndexFinger = filterPose(poseIndexFinger, poseIndexFingerOld, MINIMAL_DISTANCE, MINIMAL_STEP_DISTANCE);
        break;
    }
  }

  if (!thumbMarkerFound || !indexFingerMarkerFound || handMarker->markers.size() == 0)
  {
    ROS_ERROR_THROTTLE(1, "Marker unsichtbar!");
    return;
  }
  std_msgs::Float64 pliersDistance;
  pliersDistance.data = (poseThumb.inverse() * poseIndexFinger).translation().norm();
  pliersDistancePub.publish(pliersDistance);
  Eigen::Quaterniond interpolatedRotation =
      Eigen::Quaterniond(poseThumb.rotation()).slerp(0.5, Eigen::Quaterniond(poseIndexFinger.rotation()));
  poseAct.translate(poseThumb.translation() + (poseIndexFinger.translation() - poseThumb.translation()) / 2);
  poseAct.rotate(interpolatedRotation);

  if (initialRun)
  {
    initialPoseMarker = poseAct;
    initialRotationMarker = Eigen::Quaterniond(poseAct.rotation());
    initialRun = false;
    return;
  }
  // Überwachung, ob Bewegungsanforderung oder Rauschen
  double distance = (poseAct.translation() - initialPoseMarker.translation()).norm();

  ROS_INFO_STREAM("distance" << distance);

  if (distance >= MINIMAL_DISTANCE)
  {
    double minimalDistanceFactor =
        (distance - MINIMAL_DISTANCE) / (MINIMAL_DISTANCE + MINIMAL_STEP_DISTANCE - distance);
    if (minimalDistanceFactor > 1)
      minimalDistanceFactor = 1;
    difference =
        minimalDistanceFactor * transMotionScaling * ((poseAct.translation() - initialPoseMarker.translation()));
  }
  slerpParameter = 0;

  poseOld = poseAct;
  markerCallbackCalled = true;
  return;
}

bool MasterSlaveManipulationAbsolute::masterSlaveCallback(masterslave::Manipulation::Request &req,
                                                          masterslave::Manipulation::Response &resp)
{
  Eigen::Affine3d T_0_EE_old = Eigen::Affine3d::Identity();

  tf::poseMsgToEigen(req.T_0_EE_old, T_0_EE_old);
  if (initialRunMasterSlave)
  {
    initialPoseRobot = T_0_EE_old.translation();
    initialRotationRobot = Eigen::Quaterniond(T_0_EE_old.rotation());
    initialRunMasterSlave = false;
  }
  oldRotation = Eigen::Quaterniond(T_0_EE_old.rotation());
  Eigen::Affine3d T_0_EE_new = Eigen::Affine3d::Identity();
  masterSlaveTime = ros::Time::now().toSec() - lastMasterSlaveTime;
  lastMasterSlaveTime = ros::Time::now().toSec();
  if (!markerCallbackCalled || !thumbMarkerFound || !indexFingerMarkerFound)
  {
    resp.T_0_EE_new = req.T_0_EE_old;
    return true;
  }

  // Inkrement, da ceil(frameTime/masterSlaveTime) Zyklen gebraucht werden, um die Interpolation durchzuführen
  slerpParameter += std::ceil(masterSlaveTime / frameTime * 100 + 0.5) / 100;

  /*
   * slerpParameter hat einen Wertebereich von 0 bis 1
   * für die Rotationsinterpolation
   */
  if (slerpParameter >= 1)
    slerpParameter = 0;

  Eigen::Vector3d newTranslation = (initialPoseRobot - difference - T_0_EE_old.translation()) * slerpParameter;

  // Geschwindigkeitsüberwachung
  if (newTranslation.norm() > MAXIMUM_TRANSLATIONAL_VELOCITY * frameTime * slerpParameter)
  {
    newTranslation /= (newTranslation.norm() / (MAXIMUM_TRANSLATIONAL_VELOCITY * frameTime * slerpParameter));
  }
  T_0_EE_new.translate(T_0_EE_old.translation());
  T_0_EE_new.translate(newTranslation);

  // Rotationsskalierung
  Eigen::Quaterniond differenceRot = Eigen::Quaterniond(poseAct.rotation());
  ROS_DEBUG_STREAM(differenceRot.vec() << " \n " << differenceRot.toRotationMatrix());
  Eigen::Quaterniond differenceRotScale = Eigen::Quaterniond::Identity().slerp(rotationScaling, differenceRot);

  Eigen::Quaterniond newRotation = initialRotationRobot * differenceRotScale * initialRotationMarker.inverse();

  Eigen::AngleAxisd differenceAngle = Eigen::AngleAxisd(differenceRotScale);
  // Wenn der Winkel die maximale Winkeldifferenz überschreitet
  if (std::abs(differenceAngle.angle()) < MAXIMUM_DIFFERENCE_ANGLE_PER_STEP)
  {
    T_0_EE_new.rotate(oldRotation.slerp(slerpParameter, newRotation));
  }
  else
  {
    // Wenn nicht, dann wird die Rotation konstant auf dem alten Wert gehalten
    T_0_EE_new.rotate(oldRotation.slerp(
        slerpParameter * MAXIMUM_DIFFERENCE_ANGLE_PER_STEP / std::abs(differenceAngle.angle()), newRotation));
  }

  // Abfangen, ob der globale Kippwinkel kleiner als 90° ist

  ROS_DEBUG_STREAM("Rotation T_0_EE_new: \n" << T_0_EE_new.rotation());
  tf::poseEigenToMsg(T_0_EE_new, resp.T_0_EE_new);
  // ROS_INFO_STREAM(difference);

  return true;
}

void MasterSlaveManipulationAbsolute::cycleTimeCallback(const std_msgs::Float64ConstPtr &val)
{
  cycleTime = val->data;
}

void MasterSlaveManipulationAbsolute::configurationCallback(masterslave::MasterSlaveManipulationAbsoluteConfig &config,
                                                            uint32_t level)
{
  transMotionScaling = config.motionScaling;
  rotationScaling = config.rotationScaling;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "MasterSlaveManipulationAbsolute");
  ros::NodeHandle MasterSlaveManipulationNH;

  MasterSlaveManipulationAbsolute *manipulation = new MasterSlaveManipulationAbsolute(MasterSlaveManipulationNH);
}
