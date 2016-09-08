#include "ros/node_handle.h"
#include "tapi_iiwa.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Tapi_iiwa");
  ros::NodeHandle nh;

  Tapi_iiwa bridge(nh);
  while (ros::ok())
  {
    ros::spin();
  }

  return 0;
}
