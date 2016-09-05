#include "masterslave/manipulation/TrajectoryGenerator.h"

TrajectoryGenerator::TrajectoryGenerator(ros::NodeHandle &nh) : nh_(nh)
{
  dynamic_reconfigure::Server<masterslave::TrajectoryGeneratorConfig> server;
  dynamic_reconfigure::Server<masterslave::TrajectoryGeneratorConfig>::CallbackType f;
  f = boost::bind(&TrajectoryGenerator::configurationCallback, this, _1, _2);
  server.setCallback(f);
  cycleTimeSub = nh_.subscribe("/cycleTime", 1, &TrajectoryGenerator::cycleTimeCallback, this);
  trajectoryServer = nh_.advertiseService("/Manipulation", &TrajectoryGenerator::trajectoryCallback, this);
  ros::spin();
}

TrajectoryGenerator::~TrajectoryGenerator()
{
}

void TrajectoryGenerator::cycleTimeCallback(const std_msgs::Float64ConstPtr &val)
{
  cycleTime = val->data;
}

bool TrajectoryGenerator::trajectoryCallback(masterslave::Manipulation::Request &req,
                                             masterslave::Manipulation::Response &resp)
{
  if (start && rcmPositionThere && trajectoryGen.get() != nullptr)
  {
    Eigen::Affine3d currentPosition;
    currentPosition = trajectoryGen->calculateNextPoint();
    tf::poseEigenToMsg(currentPosition, resp.T_0_EE_new);
  }
  else
  {
    resp.T_0_EE_new = req.T_0_EE_old;
  }
  if (!rcmPositionThere)
  {
    Eigen::Affine3d startPointRCM;
    rcmPositionThere = true;
    tf::poseMsgToEigen(req.T_0_EE_old, startPointRCM);
    rcm = startPointRCM.translation();
  }
  tf::poseMsgToEigen(req.T_0_EE_old, startPosition);
  return true;
}

void TrajectoryGenerator::configurationCallback(masterslave::TrajectoryGeneratorConfig &config, uint32_t level)
{
  start = config.Start;
  state = static_cast<TRAJECTORY_STATE>(config.curTrajType);

  ;
  zCoordinate = config.groups.circle.ZCoord * MM_TO_M;
  circleRadius = config.groups.circle.Radius * MM_TO_M;
  firstPoint =
      Eigen::Vector3d(config.groups.first_point.XCoordinate * MM_TO_M, config.groups.first_point.YCoordinate * MM_TO_M,
                      config.groups.first_point.ZCoordinate * MM_TO_M);
  secondPoint = Eigen::Vector3d(config.groups.second_point.XCoordinate2 * MM_TO_M,
                                config.groups.second_point.YCoordinate2 * MM_TO_M,
                                config.groups.second_point.ZCoordinate2 * MM_TO_M);

  if (start && rcmPositionThere)
  {
    switch (state)
    {
      case PTP:
        trajectoryGen.reset(
            new PTPTrajectory(startPosition, rcm, firstPoint, secondPoint, config.Speed * MM_TO_M, cycleTime));
        break;
      case LINE:
        trajectoryGen.reset(
            new LineTrajectory(startPosition, rcm, firstPoint, secondPoint, config.Speed * MM_TO_M, cycleTime));
        break;
      case CIRCLE:
        trajectoryGen.reset(
            new CircleTrajectory(startPosition, rcm, circleRadius, zCoordinate, config.Speed * MM_TO_M, cycleTime));
        break;
    }
  }

  if (trajectoryGen.get() != nullptr)
  {
    trajectoryGen->setSpeed(config.Speed * MM_TO_M);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "TrajectoryGenerator");
  ros::NodeHandle TrajectoryGeneratorNH;
  TrajectoryGenerator *trajGen = new TrajectoryGenerator(TrajectoryGeneratorNH);
  return 0;
}
