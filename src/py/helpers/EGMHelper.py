#!/usr/bin/env python
import time
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Header
import helpers.egm_pb2 as egm_pb2

def get_tick():
    return int((time.time() + 0.5) * 1000)

def EgmFeedback_to_PoseStamped(fb):
    # EgmFeedback time stamps are useless, so current time is stamped
    posestamped = PoseStamped()
    posestamped.header = Header()
    posestamped.header.stamp = rospy.Time.now()
    posestamped.header.frame_id = "map"
    posestamped.pose = EgmFeedback_to_Pose(fb)
    return posestamped

def EgmFeedback_to_Pose(fb):
    pose = Pose()
    pose.position.x = fb.cartesian.pos.x
    pose.position.y = fb.cartesian.pos.y
    pose.position.z = fb.cartesian.pos.z
    pose.orientation.x = fb.cartesian.orient.u1
    pose.orientation.y = fb.cartesian.orient.u2
    pose.orientation.z = fb.cartesian.orient.u3
    pose.orientation.w = fb.cartesian.orient.u0
    return pose

def EgmFeedback_to_JointState(fb):
    js = JointState()
    js.header = Header()
    js.header.stamp = rospy.Time.now()
    js.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    js.position = [j for j in fb.joints.joints]
    js.velocity = [0.0 for i in xrange(6)]
    js.effort = [0.0 for i in xrange(6)]
    return js

def Pose_to_EgmSensor(pose, seqno, tick):
    header = egm_pb2.EgmHeader()
    header.mtype = egm_pb2.EgmHeader.MSGTYPE_CORRECTION
    header.seqno = seqno
    header.tm = tick

    # Positions are bounded and converted from m to mm
    pos = egm_pb2.EgmCartesian()
    pos.x = pose.position.x
    pos.y = pose.position.y
    pos.z = pose.position.z

    # Importing orientation from pose
    orient = egm_pb2.EgmQuaternion()
    orient.u0 = pose.orientation.w
    orient.u1 = pose.orientation.x
    orient.u2 = pose.orientation.y
    orient.u3 = pose.orientation.z

    # Constructing EgmSensor message to send
    egm_pose = egm_pb2.EgmPose()
    egm_pose.orient.CopyFrom(orient)
    egm_pose.pos.CopyFrom(pos)
    planned = egm_pb2.EgmPlanned()
    planned.cartesian.CopyFrom(egm_pose)
    msg = egm_pb2.EgmSensor()
    msg.header.CopyFrom(header)
    msg.planned.CopyFrom(planned)
    return msg

def translate_pose_by_velocity(pose, vel, dt):
    pose.position.x += vel.position.x * dt
    pose.position.y += vel.position.y * dt
    pose.position.z += vel.position.z * dt
    pose.orientation.x += 0.5*dt*(pose.orientation.w*vel.orientation.x - pose.orientation.y*vel.orientation.z + pose.orientation.z*vel.orientation.y)
    pose.orientation.y += 0.5*dt*(pose.orientation.w*vel.orientation.y + pose.orientation.x*vel.orientation.z - pose.orientation.z*vel.orientation.x)
    pose.orientation.z += 0.5*dt*(pose.orientation.w*vel.orientation.z + pose.orientation.x*vel.orientation.y + pose.orientation.y*vel.orientation.x)
    pose.orientation.w -= 0.5*dt*(pose.orientation.x*vel.orientation.x + pose.orientation.y*vel.orientation.y + vel.orientation.z*vel.orientation.z)
    return pose
