#include "RobotController.hpp"
#include "EGMHelper.hpp"
#include "egm.pb.h"

RobotController::RobotController(ros::NodeHandle n, int udpPort, std::string command_input, std::string ik_mode)
  : seqno(0), start_tick(get_tick()), udpPort(udpPort), ik_mode(ik_mode)
{
  // Initialize IK Solver
  // KDL::Tree kdl_tree;
  // if(!kdl_parser::treeFromFile("/root/catkin_ws/src/config/descriptions/yumi/urdf/yumi.urdf", kdl_tree)) {
  //   ROS_ERROR("Failed to construct KDL tree.");
  // }
  // ROS_INFO("No joints: %d, no. segments: %d", kdl_tree.getNrOfJoints(), kdl_tree.getNrOfSegments());
  // std::map<std::string,KDL::TreeElement> segments = kdl_tree.getSegments();
  // for(auto segment : segments) {
  //   ROS_INFO("%s", segment.first.c_str());
  // }
  //
  // KDL::Chain chain;
  // if(udpPort == 6510) {
  //   kdl_tree.getChain("yumi_base_link", "yumi_link_7_l", chain);
  // } else if(udpPort == 6513) {
  //   kdl_tree.getChain("yumi_base_link", "yumi_link_7_r", chain);
  // } else {
  //   ROS_ERROR("Wrong UDP port.");
  // }
  // KDL::JntArray lower_joint_limits = KDL::JntArray(7);
  // KDL::JntArray upper_joint_limits = KDL::JntArray(7);
  // lower_joint_limits(0) = -2.8;
  // lower_joint_limits(1) = -2.4;
  // lower_joint_limits(2) = -2.1;
  // lower_joint_limits(3) = -5.0;
  // lower_joint_limits(4) = -1.5;
  // lower_joint_limits(5) = -3.9;
  // lower_joint_limits(6) = -2.9;
  // upper_joint_limits(0) = 2.8;
  // upper_joint_limits(1) = 0.74;
  // upper_joint_limits(2) = 1.3;
  // upper_joint_limits(3) = 5.0;
  // upper_joint_limits(4) = 2.4;
  // upper_joint_limits(5) = 3.9;
  // upper_joint_limits(6) = 2.9;
  // ik_solver = new TRAC_IK::TRAC_IK(chain, lower_joint_limits, upper_joint_limits, 0.005, 1e-5, TRAC_IK::Distance);

  if(command_input == "pose") {
    if(ik_mode == "trac_ik") {
      if(udpPort == 6510)
        ik_solver = new TRAC_IK::TRAC_IK("yumi_base_link", "yumi_link_7_l", "/robot_description", 0.005, 1e-5, TRAC_IK::Distance);
      else if(udpPort == 6513)
        ik_solver = new TRAC_IK::TRAC_IK("yumi_base_link", "yumi_link_7_r", "/robot_description", 0.005, 1e-5, TRAC_IK::Distance);
      else
        ROS_ERROR("Wrong UDP port.");
    } else {
      std::string ikIp = "192.168.0.191";
      int ikPort;
      if(udpPort == 6510)
        ikPort = 5000;
      else if(udpPort == 6513)
        ikPort = 5001;
      else
        ROS_ERROR("Wrong UDP port.");

      if ((ik_socket = socket(PF_INET, SOCK_STREAM, 0)) == -1) {
        ROS_INFO("Problem creating the socket (error %d). Exiting.", errno);
      } else {
        // Now try to connect to the robot
        struct sockaddr_in remoteSocket;
        remoteSocket.sin_family = AF_INET;
        remoteSocket.sin_port = htons(ikPort);
        inet_pton(AF_INET, ikIp.c_str(), &remoteSocket.sin_addr.s_addr);
        if(connect(ik_socket, (sockaddr*)&remoteSocket, sizeof(remoteSocket)) == -1)
        {
          ROS_INFO("Problem during connection (error %d). Exiting.", errno);
        }
      }
    }
  }

  // Initialize robot connection
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

sensor_msgs::JointState RobotController::send_command(geometry_msgs::PoseStamped command_pose, std::string command_mode, double hz)
{
  new_sent_time = ros::Time::now();
  if (command_mode == "velocity") {
    if(command_pose.header.seq == lastSeq) {
      // Default behavior: send zero speed
      target = std::vector<double>(7, 0.0);
    } else {
      // TODO: IMPLEMENT THIS!
      lastSeq = command_pose.header.seq;
    }
    last_egm_sensor = Velocity_to_EgmSensor(target, last_measured_js.position, seqno++, get_tick()-start_tick);
  } else {
    if (command_pose.header.seq == lastSeq) {
      // Default behavior: send last sent pose
      target = last_sent_js.position;
    } else {
      if(ik_mode == "trac_ik") {
        // Prepare joint_seed as previous joints
        joint_seed(0) = last_sent_js.position[0]*PI/180.0;
        joint_seed(1) = last_sent_js.position[1]*PI/180.0;
        joint_seed(2) = last_sent_js.position[6]*PI/180.0;
        joint_seed(3) = last_sent_js.position[2]*PI/180.0;
        joint_seed(4) = last_sent_js.position[3]*PI/180.0;
        joint_seed(5) = last_sent_js.position[4]*PI/180.0;
        joint_seed(6) = last_sent_js.position[5]*PI/180.0;

        // Prepared command_frame
        KDL::Rotation desired_orient = KDL::Rotation::Quaternion(command_pose.pose.orientation.x, command_pose.pose.orientation.y, command_pose.pose.orientation.z, command_pose.pose.orientation.w);
        KDL::Vector desired_position =  KDL::Vector(command_pose.pose.position.x, command_pose.pose.position.y, command_pose.pose.position.z);
        KDL::Frame desired_pose = KDL::Frame(desired_orient, desired_position);

        try {
          int rc = ik_solver->CartToJnt(joint_seed, desired_pose, joint_seed);
          if(rc >= 0) {
            target = std::vector<double>(7, 0.0);
            target[0] = joint_seed(0)/PI*180.0;
            target[1] = joint_seed(1)/PI*180.0;
            target[2] = joint_seed(3)/PI*180.0;
            target[3] = joint_seed(4)/PI*180.0;
            target[4] = joint_seed(5)/PI*180.0;
            target[5] = joint_seed(6)/PI*180.0;
            target[6] = joint_seed(2)/PI*180.0;
          } else {
            target = last_sent_js.position;
            ROS_ERROR("IK failed!");
          }
        }
        catch (int e) {
          target = last_sent_js.position;
          ROS_ERROR("IK failed due to error!");
        }
      } else {
        sprintf(ik_message, "%08.1lf %08.1lf %08.1lf %08.5lf %08.5lf %08.5lf %08.5lf #", command_pose.pose.position.x, command_pose.pose.position.y, command_pose.pose.position.z, command_pose.pose.orientation.w, command_pose.pose.orientation.x, command_pose.pose.orientation.y, command_pose.pose.orientation.z);
        if (send(ik_socket, ik_message, strlen(ik_message), 0) == -1) {
          ROS_ERROR("Failed to send message. Sending last joints.");
          target = last_sent_js.position;
        } else {
          if((ik_replySize = recv(ik_socket, ik_reply, MAX_BUFFER-1, 0)) <= 0) {
            ROS_ERROR("Failed to receive message. Sending last joints.");
            target = last_sent_js.position;
          } else {
            sscanf(ik_reply, "%d %lf %lf %lf %lf %lf %lf %lf #", &ik_response, &ik_j1, &ik_j2, &ik_j3, &ik_j4, &ik_j5, &ik_j6, &ik_j7);
            switch(ik_response) {
              case 0:
                // Invalid or empty pose, no error shown
                target = last_sent_js.position;
                break;
              case 1:
                target = {ik_j1, ik_j2, ik_j3, ik_j4, ik_j5, ik_j6, ik_j7};
                break;
              case 2:
                ROS_WARN("IK: Wrong number of parameters received by robot.");
                target = last_sent_js.position;
                break;
              case 1075:
                ROS_WARN("IK: Limit error. Try another pose.");
                target = last_sent_js.position;
                break;
              case 1136:
                ROS_WARN("IK: Target is outside robot working area.");
                target = last_sent_js.position;
                break;
              default:
                ROS_WARN("IK: Error number %d.", ik_response);
                target = last_sent_js.position;
            }
          }
        }
      }
      lastSeq = command_pose.header.seq;
    }
    last_egm_sensor = Position_to_EgmSensor(target, seqno++, get_tick()-start_tick);
  }
  Target_to_JointState(target, new_sent_time, last_sent_js);
  last_egm_sensor->SerializeToString(&outBuffer);
  sock->sendTo(outBuffer.c_str(), outBuffer.length(), sourceAddr, sourcePort);
  return last_sent_js;
}
