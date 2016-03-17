#include "masterslave/manipulation/TrajectoryGenerator.h"

TrajectoryGenerator::TrajectoryGenerator(ros::NodeHandle &nh): nh_(nh)
{
    dynamic_reconfigure::Server<masterslave::TrajectoryGeneratorConfig> server;
    dynamic_reconfigure::Server<masterslave::TrajectoryGeneratorConfig>::CallbackType f;
    f = boost::bind(&TrajectoryGenerator::configurationCallback,this ,_1,_2);
    server.setCallback(f);
    cycleTimeSub = nh_.subscribe("/cycleTime",1,&TrajectoryGenerator::cycleTimeCallback,this);
    trajectoryServer = nh_.advertiseService("/Manipulation",&TrajectoryGenerator::trajectoryCallback, this);
    ros::spin();
}

TrajectoryGenerator::~TrajectoryGenerator()
{
}

void TrajectoryGenerator::cycleTimeCallback(const std_msgs::Float64ConstPtr &val)
{
    cycleTime = val->data;
}

bool TrajectoryGenerator::trajectoryCallback(masterslave::Manipulation::Request &req, masterslave::Manipulation::Response &resp)
{
    if(start && startPositionThere && trajectoryGen.get()!=nullptr)
    {
        Eigen::Affine3d currentPosition;
        currentPosition = trajectoryGen->calculateNextPoint();
        tf::poseEigenToMsg(currentPosition,resp.T_0_EE_new);
    }
    else
    {
        resp.T_0_EE_new = req.T_0_EE_old;
    }
    if(!startPositionThere)
    {
        startPositionThere = true;
        tf::poseMsgToEigen(req.T_0_EE_old,startPosition);
    }


    return true;
}

void TrajectoryGenerator::configurationCallback(masterslave::TrajectoryGeneratorConfig &config, uint32_t level)
{
    start = config.Start;
    state = static_cast<TRAJECTORY_STATE>(config.curTrajType);
    ptpTrajectory = Eigen::Vector2i(config.LengthX,config.LengthY);
    trajectorySpeed = config.Speed;
    zCoordinate = config.zCoord;
    circleRadius = config.Radius;
    if(start && startPositionThere)
    {
        switch(state)
        {
            case PTP:
                trajectoryGen.reset(new PTPTrajectory(startPosition,ptpTrajectory,zCoordinate,trajectorySpeed,cycleTime));
                break;
            case CIRCLE:
                trajectoryGen.reset(new CircleTrajectory(startPosition,circleRadius,zCoordinate,trajectorySpeed,cycleTime));
                break;
        }
    }


    if(trajectoryGen.get()!=nullptr)
    {
        trajectoryGen->setSpeed(config.Speed);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "TrajectoryGenerator");
    ros::NodeHandle TrajectoryGeneratorNH;
    TrajectoryGenerator* trajGen = new TrajectoryGenerator(TrajectoryGeneratorNH);
    return 0;
}
