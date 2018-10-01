#include "ROSHelper.hpp"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"

double x_limit_inf = 100.0;
double x_limit_sup = 600.0;
double y_limit_inf = -400.0;
double y_limit_sup = 400.0;
double z_limit_inf = 0.0;
double z_limit_sup = 500.0;

double hz = 250.0;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "EGMControl");
  ros::NodeHandle n;

  ROSHelper ros_helper = ROSHelper(n);

  ROS_INFO("[EGMControl] Ready");

  ros::Rate rate(hz);

  while (ros::ok())
  {
    geometry_msgs::PoseStamped command_pose = ros_helper.get_command_pose();
    if(command_pose.pose.position.x != 0.0)
    {
      ros_helper.publish_measured_pose(command_pose);
    }
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
