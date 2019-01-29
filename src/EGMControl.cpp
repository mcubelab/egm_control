#include "EGMHelper.hpp"
#include "ROSHelper.hpp"
#include "RobotController.hpp"

double hz = 250.0;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "EGMControl");
  ros::NodeHandle n;

  ROS_INFO("[EGMControl] Initializing...");

  int yumi_port = atoi(argv[1]);
  ROSHelper ros_helper = ROSHelper(n, yumi_port);
  RobotController robot_controller = RobotController(n, yumi_port);

  ros::Rate rate(hz);

  geometry_msgs::PoseStamped measured_pose;
  sensor_msgs::JointState command_joints, sent_joints, measured_joints;
  std::string command_mode;
  abb::egm::EgmFeedBack feedback;

  // Important: EGM mode cannot be changed after startup
  ros::param::param<std::string>("egm_command_mode", command_mode, "position");

  ROS_INFO("[EGMControl] Ready. EGM mode: %s", command_mode.c_str());

  while (ros::ok())
  {
    ros::spinOnce();
    command_joints = ros_helper.get_command_joints();
    sent_joints = robot_controller.send_command(command_joints, command_mode, hz);
    ros_helper.publish_sent_joints(sent_joints);

    try {
      robot_controller.flush_robot_data();
      robot_controller.get_measured_pose(measured_pose);
      robot_controller.get_measured_joints(measured_joints);
      ros_helper.publish_measured_pose(measured_pose);
      ros_helper.publish_measured_joints(measured_joints);
    }
    catch (SocketException& e) {
      ROS_INFO("[EGMControl] EGM reset by RAPID server");
      break;
    }

    rate.sleep();
  }

  ROS_INFO("[EGMControl] End of program");
  return 0;
}
