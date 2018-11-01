#include "RobotController.hpp"
#include "EGMHelper.hpp"
#include "egm.pb.h"

RobotController::RobotController(ros::NodeHandle n, int udpPort, limits x_limits, limits y_limits, limits z_limits)
  : seqno(0), start_tick(get_tick()), udpPort(udpPort), x_limits(x_limits), y_limits(y_limits), z_limits(z_limits)
{
  sock = new UDPSocket(udpPort);
  flush_robot_data();
  sock->setTimeout(500000);
  last_sent_ps = last_measured_ps;
}

RobotController::~RobotController()
{
  delete sock;
}

void RobotController::flush_robot_data()
{
  messageSize = sock->recvFrom(inBuffer, MAX_BUFFER-1, sourceAddr, sourcePort);
  last_egm_robot = new abb::egm::EgmRobot();

  if (messageSize < 0) {
    // TODO: Should be improved
    ROS_INFO("[EGMControl] Failed to receive message from robot");
    return;
  }
  last_egm_robot->ParseFromArray(inBuffer, messageSize);
  last_fb = last_egm_robot->feedback();
  EgmFeedBack_to_PoseStamped(&last_fb, last_measured_ps);
  EgmFeedBack_to_JointState(&last_fb, last_measured_js);
}

void RobotController::get_measured_pose(geometry_msgs::PoseStamped& posestamped)
{
  posestamped = last_measured_ps;
}

void RobotController::get_measured_js(sensor_msgs::JointState& js)
{
  js = last_measured_js;
}

geometry_msgs::PoseStamped RobotController::send_command(geometry_msgs::PoseStamped command_pose, std::string command_mode, double hz)
{
  new_sent_time = ros::Time::now();
  if (command_mode == "velocity") {
    if(command_pose.header.stamp == ros::Time(0)) {
      // Default behavior: send zero speed
      target = geometry_msgs::Pose();
    } else {
      target = command_pose.pose;
    }
    last_egm_sensor = Velocity_to_EgmSensor(target, last_measured_ps.pose, seqno++, get_tick()-start_tick);
  } else {
    if (command_pose.header.stamp == ros::Time(0)) {
      // Default behavior: send last sent pose
      target = last_sent_ps.pose;
    } else {
      target = command_pose.pose;
    }
    target.position.x = max(min(target.position.x, x_limits.second), x_limits.first);
    target.position.y = max(min(target.position.y, y_limits.second), y_limits.first);
    target.position.z = max(min(target.position.z, z_limits.second), z_limits.first);
    last_egm_sensor = Position_to_EgmSensor(target, seqno++, get_tick()-start_tick);
  }
  Pose_to_PoseStamped(target, new_sent_time, last_sent_ps);
  last_egm_sensor->SerializeToString(&outBuffer);
  sock->sendTo(outBuffer.c_str(), outBuffer.length(), sourceAddr, sourcePort);
  return last_sent_ps;
}
