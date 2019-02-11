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

#include <urdf/model.h>
#include <kdl/kdl.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <trac_ik/trac_ik.hpp>

#include <vector>
#include <map>
#include <string>

#define PI 3.14159265

class RobotController {
public:
  RobotController(ros::NodeHandle n, int udpPort, std::string command_input);
  ~RobotController();

  void flush_robot_data();
  void get_measured_pose(geometry_msgs::PoseStamped& posestamped);
  void get_measured_joints(sensor_msgs::JointState& js);

  abb::egm::EgmFeedBack get_robot_feedback();
  sensor_msgs::JointState send_command(sensor_msgs::JointState command_joints, std::string command_mode, double hz);
  sensor_msgs::JointState send_command(geometry_msgs::PoseStamped command_pose, std::string command_mode, double hz);

private:
  unsigned int seqno;
  uint32_t start_tick;
  UDPSocket* sock;
  int udpPort;
  const static int MAX_BUFFER = 1400;
  char inBuffer[MAX_BUFFER];
  std::string outBuffer;
  int messageSize;
  int lastSeq;
  std::string sourceAddr;
  unsigned short sourcePort;

  geometry_msgs::PoseStamped last_measured_ps;
  sensor_msgs::JointState last_measured_js, last_sent_js;
  ros::Time new_sent_time;
  std::vector<double> target;
  abb::egm::EgmFeedBack last_fb;
  abb::egm::EgmRobot* last_egm_robot;
  abb::egm::EgmSensor* last_egm_sensor;

  KDL::JntArray joint_seed = KDL::JntArray(7);
  TRAC_IK::TRAC_IK* ik_solver;
};

#endif
