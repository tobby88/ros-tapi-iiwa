#include "masterslave/manipulation/trajectorygenerator.h"

TrajectoryGenerator::TrajectoryGenerator(ros::NodeHandle &nh): nh_(nh)
{
    dynamic_reconfigure::Server<masterslave::trajectorygeneratorConfig> server;
    dynamic_reconfigure::Server<masterslave::trajectorygeneratorConfig>::CallbackType f;
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
    if(!start)
    {
        resp.T_0_EE_new = req.T_0_EE_old;
        return true;
    }

    if(start && !startOld)
    {
        Eigen::Affine3d T_0_EE;
        tf::poseMsgToEigen(req.T_0_EE_old,T_0_EE);
        switch(state)
        {
            case PTP:
                trajectoryGen = std::move(std::unique_ptr<PTPTraj>(new PTPTraj(T_0_EE,ptpTrajectory,zCoordinate,trajectorySpeed,cycleTime)));
                break;
            case CIRCLE:
                trajectoryGen = std::move(std::unique_ptr<CircleTraj>(new CircleTraj(T_0_EE,circleRadius,zCoordinate,trajectorySpeed,cycleTime)));
                break;
        }
    }
    if(!start && startOld)
    {
        trajectoryGen.release();
        ROS_INFO("RELEASED");
    }
    ROS_INFO_STREAM(state);
    Eigen::Affine3d T_0_EE;
    T_0_EE = trajectoryGen->calculateNextPoint();
    tf::poseEigenToMsg(T_0_EE,resp.T_0_EE_new);

    startOld = start;
}

void TrajectoryGenerator::configurationCallback(masterslave::trajectorygeneratorConfig &config, uint32_t level)
{

    start = config.Start;
    state = static_cast<TRAJECTORY_STATE>(config.curTrajType);
    ptpTrajectory = Eigen::Vector2i(config.LengthX,config.LengthY);
    trajectorySpeed = config.Speed;
    zCoordinate = config.zCoord;
    circleRadius = config.Radius;
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
