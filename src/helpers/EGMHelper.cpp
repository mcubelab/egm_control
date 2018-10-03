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

geometry_msgs::PoseStamped Pose_to_PoseStamped(geometry_msgs::Pose pose, ros::Time time)
{
  geometry_msgs::PoseStamped posestamped;
  posestamped.header.stamp = time;
  posestamped.header.frame_id = "map";
  posestamped.pose = pose;
  return posestamped;
}

geometry_msgs::Pose EgmFeedBack_to_Pose(abb::egm::EgmFeedBack *fb)
{
  geometry_msgs::Pose pose;
  pose.position.x = fb->cartesian().pos().x();
  pose.position.y = fb->cartesian().pos().y();
  pose.position.z = fb->cartesian().pos().z();
  pose.orientation.x = fb->cartesian().orient().u1();
  pose.orientation.y = fb->cartesian().orient().u2();
  pose.orientation.z = fb->cartesian().orient().u3();
  pose.orientation.w = fb->cartesian().orient().u0();
  return pose;
}

geometry_msgs::PoseStamped EgmFeedBack_to_PoseStamped(abb::egm::EgmFeedBack *fb)
{
  geometry_msgs::PoseStamped posestamped;
  posestamped.header.stamp = ros::Time::now();
  posestamped.header.frame_id = "map";
  posestamped.pose = EgmFeedBack_to_Pose(fb);
  return posestamped;
}

sensor_msgs::JointState EgmFeedBack_to_JointState(abb::egm::EgmFeedBack *fb)
{
  sensor_msgs::JointState js;
  js.header.stamp = ros::Time::now();
  js.name.push_back("joint1");
  js.name.push_back("joint2");
  js.name.push_back("joint3");
  js.name.push_back("joint4");
  js.name.push_back("joint5");
  js.name.push_back("joint6");
  for (int i = 0; i < 6; i++) {
    js.position.push_back(fb->joints().joints(i));
    js.velocity.push_back(0.0);
    js.effort.push_back(0.0);
  }
  return js;
}

abb::egm::EgmSensor* Pose_to_EgmSensor(geometry_msgs::Pose pose, uint32_t seqno, uint32_t tick)
{
  abb::egm::EgmHeader* header = new abb::egm::EgmHeader();
  header->set_mtype(abb::egm::EgmHeader_MessageType_MSGTYPE_CORRECTION);
  header->set_seqno(seqno);
  header->set_tm(tick);

  abb::egm::EgmCartesian *pc = new abb::egm::EgmCartesian();
  pc->set_x(pose.position.x);
  pc->set_y(pose.position.y);
  pc->set_z(pose.position.z);

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
  msg->set_allocated_planned(planned);
  return msg;
}

geometry_msgs::Pose translate_pose_by_velocity(geometry_msgs::Pose pose, geometry_msgs::Pose vel, double dt)
{
  pose.position.x += vel.position.x * dt;
  pose.position.y += vel.position.y * dt;
  pose.position.z += vel.position.z * dt;
  pose.orientation.x += 0.5*dt*(pose.orientation.w*vel.orientation.x - pose.orientation.y*vel.orientation.z + pose.orientation.z*vel.orientation.y);
  pose.orientation.y += 0.5*dt*(pose.orientation.w*vel.orientation.y + pose.orientation.x*vel.orientation.z - pose.orientation.z*vel.orientation.x);
  pose.orientation.z += 0.5*dt*(pose.orientation.w*vel.orientation.z + pose.orientation.x*vel.orientation.y + pose.orientation.y*vel.orientation.x);
  pose.orientation.w -= 0.5*dt*(pose.orientation.x*vel.orientation.x + pose.orientation.y*vel.orientation.y + vel.orientation.z*vel.orientation.z);
  return pose;
}