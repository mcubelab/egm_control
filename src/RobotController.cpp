#include "RobotController.hpp"
#include "EGMHelper.hpp"
#include "egm.pb.h"

RobotController::RobotController(ros::NodeHandle n, int udpPort)
  : seqno(0), start_tick(get_tick()), udpPort(udpPort)
{
  sock = new UDPSocket(udpPort);
  flush_robot_data();
  sock->setTimeout(1000000);
  last_sent_js = last_measured_js;
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

void RobotController::get_measured_joints(sensor_msgs::JointState& js)
{
  js = last_measured_js;
}

sensor_msgs::JointState RobotController::send_command(sensor_msgs::JointState command_joints, std::string command_mode, double hz)
{
  new_sent_time = ros::Time::now();
  if (command_mode == "velocity") {
    if(command_joints.header.seq == lastSeq) {
      // Default behavior: send zero speed
      target = std::vector<double>(7, 0.0);
    } else {
      target = command_joints.velocity;
      lastSeq = command_joints.header.seq;
    }
    last_egm_sensor = Velocity_to_EgmSensor(target, last_measured_js.position, seqno++, get_tick()-start_tick);
  } else {
    if (command_joints.header.seq == lastSeq) {
      // Default behavior: send last sent pose
      target = last_sent_js.position;
    } else {
      target = command_joints.position;
      lastSeq = command_joints.header.seq;
    }
    last_egm_sensor = Position_to_EgmSensor(target, seqno++, get_tick()-start_tick);
  }
  Target_to_JointState(target, new_sent_time, last_sent_js);
  last_egm_sensor->SerializeToString(&outBuffer);
  sock->sendTo(outBuffer.c_str(), outBuffer.length(), sourceAddr, sourcePort);
  return last_sent_js;
}
