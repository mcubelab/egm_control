#ifndef _EGMCONTROL_ROBOTCONTROLLER_HPP_
#define _EGMCONTROL_ROBOTCONTROLLER_HPP_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "PracticalSocket.h"
#include "egm.pb.h"

#include <vector>
#include <string>

typedef std::pair<double, double> limits;

class RobotController {
public:
  RobotController(ros::NodeHandle n, int udpPort, limits x_limits, limits y_limits, limits z_limits);
  ~RobotController();

  abb::egm::EgmFeedBack get_robot_feedback();
  geometry_msgs::PoseStamped send_command(geometry_msgs::PoseStamped command_pose, std::string command_mode, double hz);

private:
  unsigned int seqno;
  uint32_t start_tick;
  UDPSocket* sock;
  int udpPort;
  const static int MAX_BUFFER = 1400;
  char inBuffer[MAX_BUFFER];
  std::string outBuffer;
  int messageSize;
  std::string sourceAddr;
  unsigned short sourcePort;
  limits x_limits;
  limits y_limits;
  limits z_limits;

  geometry_msgs::PoseStamped last_measured_ps;
  geometry_msgs::PoseStamped last_sent_ps;
  ros::Time new_sent_time;
  geometry_msgs::Pose target;
  abb::egm::EgmRobot* last_egm_robot;
  abb::egm::EgmSensor* last_egm_sensor;
};

#endif
