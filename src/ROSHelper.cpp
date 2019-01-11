#include "ROSHelper.hpp"

ROSHelper::ROSHelper(ros::NodeHandle n)
{
  command_joints_sub = n.subscribe("/command_joints", 100, &ROSHelper::load_command_joints, this);
  measured_joints_pub = n.advertise<sensor_msgs::JointState>("measured_joints", 100);
  measured_pose_pub = n.advertise<geometry_msgs::PoseStamped>("measured_pose", 100);
  sent_joints_pub = n.advertise<sensor_msgs::JointState>("sent_joints", 100);
}

ROSHelper::~ROSHelper()
{
}

void ROSHelper::load_command_joints(const sensor_msgs::JointState& data)
{
  command_joints = data;
}

sensor_msgs::JointState ROSHelper::get_command_joints()
{
  return command_joints;
}

void ROSHelper::publish_measured_joints(const sensor_msgs::JointState joints)
{
  measured_joints_pub.publish(joints);
}

void ROSHelper::publish_measured_pose(const geometry_msgs::PoseStamped pose)
{
  measured_pose_pub.publish(pose);
}

void ROSHelper::publish_sent_joints(const sensor_msgs::JointState joints)
{
  sent_joints_pub.publish(joints);
}
