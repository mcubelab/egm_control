#ifndef _EGMCONTROL_EGMHELPER_HPP_
#define _EGMCONTROL_EGMHELPER_HPP_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "egm.pb.h"

#include <vector>
#include <string>

uint32_t get_tick();

void Pose_to_PoseStamped(const geometry_msgs::Pose& pose, ros::Time time, geometry_msgs::PoseStamped& posestamped);

void EgmFeedBack_to_Pose(abb::egm::EgmFeedBack *fb, geometry_msgs::Pose& pose);

void EgmFeedBack_to_PoseStamped(abb::egm::EgmFeedBack *fb, geometry_msgs::PoseStamped& posestamped);

void EgmFeedBack_to_JointState(abb::egm::EgmFeedBack *fb, sensor_msgs::JointState& js);

abb::egm::EgmSensor* Position_to_EgmSensor(geometry_msgs::Pose pose, uint32_t seqno, uint32_t tick);

abb::egm::EgmSensor* Velocity_to_EgmSensor(geometry_msgs::Pose vel, uint32_t seqno, uint32_t tick);

void translate_pose_by_velocity(geometry_msgs::Pose pose, geometry_msgs::Pose vel, double dt, geometry_msgs::Pose& target);

#endif
