#include "RobotController.hpp"
#include "EGMHelper.hpp"
#include "egm.pb.h"

RobotController::RobotController(ros::NodeHandle n, int udpPort, limits x_limits, limits y_limits, limits z_limits)
  : seqno(0), start_tick(get_tick()), udpPort(udpPort), x_limits(x_limits), y_limits(y_limits), z_limits(z_limits)
{
  sock = new UDPSocket(udpPort);
  abb::egm::EgmFeedBack fb = this->get_robot_feedback();
  last_measured_ps = EgmFeedBack_to_PoseStamped(&fb);
  last_sent_ps = last_measured_ps;
}

RobotController::~RobotController()
{
  delete sock;
}

abb::egm::EgmFeedBack RobotController::get_robot_feedback()
{
  messageSize = sock->recvFrom(inBuffer, MAX_BUFFER-1, sourceAddr, sourcePort);
  abb::egm::EgmRobot* msg = new abb::egm::EgmRobot();

  if (messageSize < 0) {
    // TODO: Should be improved
    ROS_INFO("[EGMControl] Failed to receive message from robot");
    return msg->feedback();
  }
  msg->ParseFromArray(inBuffer, messageSize);
  return msg->feedback();
}

geometry_msgs::PoseStamped RobotController::send_command(geometry_msgs::PoseStamped command_pose, std::string command_mode, double hz)
{
  ros::Time new_sent_time = ros::Time::now();
  geometry_msgs::Pose target;
  if (command_pose.header.stamp == ros::Time(0)) {
    if (command_mode == "velocity")
      target = last_sent_ps.pose;
    else
      target = last_measured_ps.pose;
  } else {
    if (command_mode == "velocity") {
      double dt = (seqno > 0 ? new_sent_time.toSec()-last_sent_ps.header.stamp.toSec() : 1/hz);
      target = translate_pose_by_velocity(last_sent_ps.pose, command_pose.pose, dt);
    } else {
      target = command_pose.pose;
    }
  }

  target.position.x = max(min(target.position.x, x_limits.second), x_limits.first);
  target.position.y = max(min(target.position.y, y_limits.second), y_limits.first);
  target.position.z = max(min(target.position.z, z_limits.second), z_limits.first);

  abb::egm::EgmSensor* msg = Pose_to_EgmSensor(target, seqno++, get_tick()-start_tick);
  last_sent_ps = Pose_to_PoseStamped(target, new_sent_time);
  msg->SerializeToString(&outBuffer);
  sock->sendTo(outBuffer.c_str(), outBuffer.length(), sourceAddr, sourcePort);
  return last_sent_ps;
}
