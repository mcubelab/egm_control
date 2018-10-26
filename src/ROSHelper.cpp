#include "ROSHelper.hpp"

ROSHelper::ROSHelper(ros::NodeHandle n)
{
  command_pose_sub = n.subscribe("/command_pose", 100, &ROSHelper::load_command_pose, this);
  joint_state_pub = n.advertise<sensor_msgs::JointState>("joint_states", 100);
  measured_pose_pub = n.advertise<geometry_msgs::PoseStamped>("measured_pose", 100);
  sent_pose_pub = n.advertise<geometry_msgs::PoseStamped>("sent_pose", 100);
  command_poses = std::vector<geometry_msgs::PoseStamped>();
  max_queued = 0;
}

ROSHelper::~ROSHelper()
{
  command_poses.clear();
}

void ROSHelper::load_command_pose(const geometry_msgs::PoseStamped& data)
{
  command_poses.push_back(data);
  if(command_poses.size() > max_queued) {
    ROS_INFO("[EGMControl] New maximum of command poses in buffer: %d", max_queued);
    max_queued = command_poses.size();
  }
}

geometry_msgs::PoseStamped ROSHelper::get_command_pose()
{
  if(command_poses.size() > 0) {
    last_command_ps = command_poses[0];
    command_poses.erase(command_poses.begin());
    return last_command_ps;
  } else {
    // ROS_INFO("[EGMControl] Lack of command poses in buffer");
    return geometry_msgs::PoseStamped();
  }
}

void ROSHelper::publish_joint_state(const sensor_msgs::JointState joints)
{
  joint_state_pub.publish(joints);
}

void ROSHelper::publish_measured_pose(const geometry_msgs::PoseStamped pose)
{
  measured_pose_pub.publish(pose);
}

void ROSHelper::publish_sent_pose(const geometry_msgs::PoseStamped pose)
{
  sent_pose_pub.publish(pose);
}

int ROSHelper::get_max_queued()
{
  return max_queued;
}
