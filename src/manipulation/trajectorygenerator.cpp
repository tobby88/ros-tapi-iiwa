#include "masterslave/manipulation/trajectorygenerator.h"

TrajectoryGenerator::TrajectoryGenerator(ros::NodeHandle &nh): nh_(nh)
{
    dynamic_reconfigure::Server<masterslave::trajectorygeneratorConfig> server;
    dynamic_reconfigure::Server<masterslave::trajectorygeneratorConfig>::CallbackType f;
    f = boost::bind(&TrajectoryGenerator::configurationCallback,this ,_1,_2);
    server.setCallback(f);
    trajectoryServer = nh_.advertiseService("/Manipulation",&TrajectoryGenerator::trajectoryCallback, this);
    ros::spin();
}

void TrajectoryGenerator::cycleTimeCallback(const std_msgs::Float64ConstPtr &val)
{
    cycleTime = val->data;
}

bool TrajectoryGenerator::trajectoryCallback(masterslave::Manipulation::Request &req, masterslave::Manipulation::Response &resp)
{
    if(!start)
    {
        req.T_0_EE_old = resp.T_0_EE_new;
        return false;
    }
    if(start && !startOld)
    {
        Eigen::Affine3d T_0_EE;
        tf::poseMsgToEigen(req.T_0_EE_old,T_0_EE);
        trajectoryGen = new PTPTraj(T_0_EE,trajectory,cycleTime);
    }
    switch(state)
    {
        case PTP:
            Eigen::Affine3d T_0_EE = trajectoryGen->calculateNextPoint();
            tf::poseEigenToMsg(T_0_EE,resp.T_0_EE_new);
            break;
    }
}

void TrajectoryGenerator::configurationCallback(masterslave::trajectorygeneratorConfig &config, uint32_t level)
{
    trajectory = config;
    start = config.Start;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "TrajectoryGenerator");
    ros::NodeHandle TrajectoryGeneratorNH;
    TrajectoryGenerator* trajGen = new TrajectoryGenerator(TrajectoryGeneratorNH);
    return 0;
}
