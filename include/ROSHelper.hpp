#ifndef _EGMCONTROL_ROSHELPER_HPP_
#define _EGMCONTROL_ROSHELPER_HPP_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"

#include <vector>

class ROSHelper {
public:
  ROSHelper(ros::NodeHandle n);

  ~ROSHelper();

  void load_command_joints(const sensor_msgs::JointState& data);

  sensor_msgs::JointState get_command_joints();

  void publish_measured_joints(const sensor_msgs::JointState joints);

  void publish_measured_pose(const geometry_msgs::PoseStamped pose);

  void publish_sent_joints(const sensor_msgs::JointState joints);

private:
  ros::Subscriber command_joints_sub;
  ros::Publisher measured_joints_pub;
  ros::Publisher measured_pose_pub;
  ros::Publisher sent_joints_pub;
  sensor_msgs::JointState command_joints;
};

#endif
