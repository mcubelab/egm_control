#include "EGMHelper.hpp"
#include "ROSHelper.hpp"
#include "RobotController.hpp"

limits x_limits = limits(0.1, 0.6);
limits y_limits = limits(-0.4, 0.4);
limits z_limits = limits(0.0, 0.5);

double hz = 250.0;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "EGMControl");
  ros::NodeHandle n;

  ROSHelper ros_helper = ROSHelper(n);
  RobotController robot_controller = RobotController(n, 6510, x_limits, y_limits, z_limits);

  ros::Rate rate(hz);

  geometry_msgs::PoseStamped command_pose, sent_pose, measured_pose;
  sensor_msgs::JointState joint_state;
  std::string command_mode;
  abb::egm::EgmFeedBack feedback;
  ros::param::param<std::string>("egm_mode", command_mode, "position");

  ROS_INFO("[EGMControl] Ready");

  while (ros::ok())
  {
    ros::spinOnce();
    command_pose = ros_helper.get_command_pose();
    sent_pose = robot_controller.send_command(command_pose, command_mode, hz);
    ros_helper.publish_sent_pose(sent_pose);

    robot_controller.flush_robot_data();
    robot_controller.get_measured_pose(measured_pose);
    robot_controller.get_measured_js(joint_state);
    ros_helper.publish_measured_pose(measured_pose);
    ros_helper.publish_joint_state(joint_state);

    rate.sleep();
  }
  delete &ros_helper;
  delete &robot_controller;
  ROS_INFO("[EGMControl] End of program");
  return 0;
}
