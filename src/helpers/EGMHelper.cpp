#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "egm.pb.h"

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
  pose.position.x = fb->cartesian().pos().x()/1000.0;
  pose.position.y = fb->cartesian().pos().y()/1000.0;
  pose.position.z = fb->cartesian().pos().z()/1000.0;
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
  js.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
  js.position.resize(6);
  for (int i = 0; i < 6; i++)
    js.position[i] = fb->joints().joints(i);
  js.velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  js.effort = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
}

abb::egm::EgmSensor* Position_to_EgmSensor(geometry_msgs::Pose pose, unsigned int seqno, uint32_t tick)
{
  abb::egm::EgmHeader* header = new abb::egm::EgmHeader();
  header->set_mtype(abb::egm::EgmHeader_MessageType_MSGTYPE_CORRECTION);
  header->set_seqno(seqno);
  header->set_tm(tick);

  abb::egm::EgmCartesian *pc = new abb::egm::EgmCartesian();
  pc->set_x(pose.position.x*1000.0);
  pc->set_y(pose.position.y*1000.0);
  pc->set_z(pose.position.z*1000.0);

  abb::egm::EgmQuaternion *pq = new abb::egm::EgmQuaternion();
  pq->set_u0(pose.orientation.w);
  pq->set_u1(pose.orientation.x);
  pq->set_u2(pose.orientation.y);
  pq->set_u3(pose.orientation.z);

  abb::egm::EgmPose *pp = new abb::egm::EgmPose();
  pp->set_allocated_pos(pc);
  pp->set_allocated_orient(pq);

  abb::egm::EgmPlanned *planned = new abb::egm::EgmPlanned();
  planned->set_allocated_cartesian(pp);

  abb::egm::EgmSensor* msg = new abb::egm::EgmSensor();
  msg->set_allocated_header(header);
  msg->set_allocated_planned(planned);
  return msg;
}

abb::egm::EgmSensor* Velocity_to_EgmSensor(geometry_msgs::Pose vel, unsigned int seqno, uint32_t tick)
{
  abb::egm::EgmHeader* header = new abb::egm::EgmHeader();
  header->set_mtype(abb::egm::EgmHeader_MessageType_MSGTYPE_CORRECTION);
  header->set_seqno(seqno);
  header->set_tm(tick);

  abb::egm::EgmCartesianSpeed *cs = new abb::egm::EgmCartesianSpeed();
  cs->add_value(vel.position.x*1000.0);
  cs->add_value(vel.position.y*1000.0);
  cs->add_value(vel.position.z*1000.0);
  cs->add_value(vel.orientation.x);
  cs->add_value(vel.orientation.y);
  cs->add_value(vel.orientation.z);

  abb::egm::EgmSpeedRef *speedref = new abb::egm::EgmSpeedRef();
  speedref->set_allocated_cartesians(cs);

  abb::egm::EgmSensor* msg = new abb::egm::EgmSensor();
  msg->set_allocated_header(header);
  msg->set_allocated_speedref(speedref);
  return msg;
}

void translate_pose_by_velocity(geometry_msgs::Pose pose, geometry_msgs::Pose vel, double dt, geometry_msgs::Pose& target)
{
  target.position.x += vel.position.x * dt;
  target.position.y += vel.position.y * dt;
  target.position.z += vel.position.z * dt;
  target.orientation.x += 0.5*dt*(pose.orientation.w*vel.orientation.x - pose.orientation.y*vel.orientation.z + pose.orientation.z*vel.orientation.y);
  target.orientation.y += 0.5*dt*(pose.orientation.w*vel.orientation.y + pose.orientation.x*vel.orientation.z - pose.orientation.z*vel.orientation.x);
  target.orientation.z += 0.5*dt*(pose.orientation.w*vel.orientation.z + pose.orientation.x*vel.orientation.y + pose.orientation.y*vel.orientation.x);
  target.orientation.w -= 0.5*dt*(pose.orientation.x*vel.orientation.x + pose.orientation.y*vel.orientation.y + vel.orientation.z*vel.orientation.z);
}
