#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "egm.pb.h"

#define PI 3.14159265
#define RAD2DEG 180/PI

#pragma comment(lib, "libprotobuf.lib")

uint32_t get_tick()
{
  struct timespec now;
  if (clock_gettime(CLOCK_MONOTONIC, &now))
    return 0;
  return now.tv_sec * 1000 + now.tv_nsec / 1000000;
}

void Pose_to_PoseStamped(const geometry_msgs::Pose& pose, ros::Time time, geometry_msgs::PoseStamped& posestamped)
{
  posestamped.header.stamp = time;
  posestamped.header.frame_id = "world";
  posestamped.pose = pose;
}

void EgmFeedBack_to_Pose(abb::egm::EgmFeedBack *fb, geometry_msgs::Pose& pose)
{
  pose.position.x = fb->cartesian().pos().x();
  pose.position.y = fb->cartesian().pos().y();
  pose.position.z = fb->cartesian().pos().z();
  pose.orientation.x = fb->cartesian().orient().u1();
  pose.orientation.y = fb->cartesian().orient().u2();
  pose.orientation.z = fb->cartesian().orient().u3();
  pose.orientation.w = fb->cartesian().orient().u0();
}

void EgmFeedBack_to_PoseStamped(abb::egm::EgmFeedBack *fb, geometry_msgs::PoseStamped& posestamped)
{
  posestamped.header.stamp = ros::Time::now();
  posestamped.header.frame_id = "world";
  EgmFeedBack_to_Pose(fb, posestamped.pose);
}

void EgmFeedBack_to_JointState(abb::egm::EgmFeedBack *fb, sensor_msgs::JointState& js)
{
  js.header.stamp = ros::Time::now();
  js.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
  js.position.resize(7);
  for (int i = 0; i < 6; i++)
    js.position[i] = fb->joints().joints(i)*RAD2DEG;
  js.position[6] =  fb->externaljoints().joints(0)*RAD2DEG;
}

abb::egm::EgmSensor* Position_to_EgmSensor(std::vector<double> target, unsigned int seqno, uint32_t tick)
{
  abb::egm::EgmHeader* header = new abb::egm::EgmHeader();
  header->set_mtype(abb::egm::EgmHeader_MessageType_MSGTYPE_CORRECTION);
  header->set_seqno(seqno);
  header->set_tm(tick);

  abb::egm::EgmJoints *pj = new abb::egm::EgmJoints();
  pj->add_joints(target[0]);
  pj->add_joints(target[1]);
  pj->add_joints(target[2]);
  pj->add_joints(target[3]);
  pj->add_joints(target[4]);
  pj->add_joints(target[5]);

  abb::egm::EgmJoints *pej = new abb::egm::EgmJoints();
  pej->add_joints(target[6]);

  abb::egm::EgmPlanned *planned = new abb::egm::EgmPlanned();
  planned->set_allocated_joints(pj);
  planned->set_allocated_externaljoints(pej);

  abb::egm::EgmSensor* msg = new abb::egm::EgmSensor();
  msg->set_allocated_header(header);
  msg->set_allocated_planned(planned);
  return msg;
}

abb::egm::EgmSensor* Velocity_to_EgmSensor(std::vector<double> target, std::vector<double> pos, unsigned int seqno, uint32_t tick)
{
  abb::egm::EgmJoints *pj = new abb::egm::EgmJoints();
  pj->add_joints(target[0]);
  pj->add_joints(target[1]);
  pj->add_joints(target[2]);
  pj->add_joints(target[3]);
  pj->add_joints(target[4]);
  pj->add_joints(target[5]);

  abb::egm::EgmJoints *pej = new abb::egm::EgmJoints();
  pej->add_joints(target[6]);

  abb::egm::EgmSpeedRef *speedref = new abb::egm::EgmSpeedRef();
  speedref->set_allocated_joints(pj);
  speedref->set_allocated_externaljoints(pej);

  // A valid position must be sent too, although it is ignored
  abb::egm::EgmSensor* msg = Position_to_EgmSensor(pos, seqno, tick);
  msg->set_allocated_speedref(speedref);
  return msg;
}

void Target_to_JointState(const std::vector<double>& target, ros::Time time, sensor_msgs::JointState& js)
{
  js.header.stamp = time;
  js.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
  js.position = target;
}
