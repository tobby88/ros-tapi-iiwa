#include "masterslave/manipulation/VisualServoing.h"

VisualServoing::VisualServoing(ros::NodeHandle &nh): nh_(nh)
{
    markerSub = nh_.subscribe("/ar_pose_marker",1,&VisualServoing::markerCallback,this);
    //visualServoingServer = nh_.advertiseService("/Manipulation",&VisualServoing::visualServoingCallback,this);
    ros::spin();
}

bool VisualServoing::visualServoingCallback(masterslave::Manipulation::Request &req, masterslave::Manipulation::Response &resp)
{

}

void VisualServoing::markerCallback(const ar_track_alvar_msgs::AlvarMarkersConstPtr& markerPtr)
{
    int markerFound = 0;
    Eigen::Affine3d tcp;
    Eigen::Affine3d obj;
    for(int i=0; i< markerPtr->markers.size(); i++)
    {
        switch(markerPtr->markers[i].id)
        {
            case 0:
            tf::poseMsgToEigen(markerPtr->markers[i].pose.pose,tcp);
            markerFound += 1;
            break;
            case 1:
            tf::poseMsgToEigen(markerPtr->markers[i].pose.pose,obj);
            markerFound += 1 << 1;
            break;
        }
    }
    if(markerFound+1 >> 2 == 1)
    {
        actualTransform = obj*tcp.inverse();
        if(initialRun)
        {
            initialTransform = actualTransform;
            initialRun = false;
        }
    }
    differenceTransform = actualTransform*initialTransform.inverse();
    ROS_INFO_STREAM("actual Transform" << actualTransform.matrix() << "\n difference Transform" << differenceTransform.matrix());
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"VisualServoing");
    ros::NodeHandle VisualServoingNH;
    VisualServoing* visualServoing = new VisualServoing(VisualServoingNH);
}
