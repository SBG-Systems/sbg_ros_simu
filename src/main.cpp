// ROS headers
#include <ros/ros.h>

// Project headers
#include <sbg_simu.h>

using sbg::SbgSimu;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sbg_simu");
  ros::NodeHandle node_handle;

  try
  {
    ROS_INFO("SBG SIMU - Initialising");

    SbgSimu sbg_simu(node_handle);

    ros::Subscriber subImu = node_handle.subscribe("/imu", 1, &SbgSimu::imuCallback, &sbg_simu);
    ros::Subscriber subFix = node_handle.subscribe("/fix", 1, &SbgSimu::gpsCallback, &sbg_simu);

    ros::spin();
  }
  catch (ros::Exception const& refE)
  {
    ROS_ERROR("SBG_SIMU - %s", refE.what());
  }

  return 0;
}
