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
  ROSHelper(ros::NodeHandle n, int yumi_port);

  ~ROSHelper();

  void load_command_joints(const sensor_msgs::JointState& data);

  sensor_msgs::JointState get_command_joints();

  void load_command_pose(const geometry_msgs::PoseStamped& data);

  geometry_msgs::PoseStamped get_command_pose();

  void publish_measured_joints(const sensor_msgs::JointState joints);

  void publish_measured_pose(const geometry_msgs::PoseStamped pose);

  void publish_sent_joints(const sensor_msgs::JointState joints);

private:
  ros::Subscriber command_joints_sub, command_pose_sub;
  ros::Publisher measured_joints_pub, measured_pose_pub, sent_joints_pub;
  sensor_msgs::JointState command_joints;
  geometry_msgs::PoseStamped command_pose;
};

#endif
