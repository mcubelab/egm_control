#!/usr/bin/env python
import time
import rospy
import socket
import helpers.egm_pb2 as egm_pb2
import helpers.egm_helper as egm_helper
from geometry_msgs.msg import PoseStamped

class RobotController():

    def __init__(self, x_limits, y_limits, z_limits):
        self.UDP_PORT = 6510
        self.sequenceNumber = 0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("", self.UDP_PORT))
        data, self.addr = self.sock.recvfrom(1024)
        rospy.loginfo('[EGMControl] Successfully connected to robot. IP: %s', self.addr[0])
        self.lastfb = self.get_feedback_from_data(data)
        rospy.loginfo('[EGMControl] Initial position (x, y, z) = (%.2f, %.2f, %.2f) mm', self.lastfb.cartesian.pos.x, self.lastfb.cartesian.pos.y, self.lastfb.cartesian.pos.z)
        rospy.loginfo('[EGMControl] Initial position (w, x, y, z) = (%d, %d, %d, %d)', self.lastfb.cartesian.orient.u0, self.lastfb.cartesian.orient.u1, self.lastfb.cartesian.orient.u2, self.lastfb.cartesian.orient.u3)
        self.starttick = egm_helper.get_tick()
        self.x_limits = x_limits
        self.y_limits = y_limits
        self.z_limits = z_limits

    def __del__(self):
        self.sock.close()

    def get_feedback_from_data(self, data):
        egm_robot = egm_pb2.EgmRobot()
        egm_robot.ParseFromString(data)
        return egm_robot.feedBack

    def get_robot_feedback(self):
        try:
            data, addr = self.sock.recvfrom(1024)
            fb = self.get_feedback_from_data(data)
            self.lastfb = fb
            self.debug_measured_pose(fb)
            return fb
        except Exception as e:
            return None

    def set_robot_pose(self, pose):
        # Preparing message header
        header = egm_pb2.EgmHeader()
        header.mtype = egm_pb2.EgmHeader.MSGTYPE_CORRECTION
        header.seqno = self.sequenceNumber
        header.tm = egm_helper.get_tick()-self.starttick
        self.sequenceNumber += 1

        # Positions are bounded and converted from m to mm
        pos = egm_pb2.EgmCartesian()
        pos.x = max(min(pose.pose.position.x, self.x_limits[1]), self.x_limits[0])
        pos.y = max(min(pose.pose.position.y, self.y_limits[1]), self.y_limits[0])
        pos.z = max(min(pose.pose.position.z, self.z_limits[1]), self.z_limits[0])

        # Importing orientation from pose
        orient = egm_pb2.EgmQuaternion()
        orient.u0 = pose.pose.orientation.w
        orient.u1 = pose.pose.orientation.x
        orient.u2 = pose.pose.orientation.y
        orient.u3 = pose.pose.orientation.z

        # Constructing EgmSensor message to send
        pose = egm_pb2.EgmPose()
        pose.orient.CopyFrom(orient)
        pose.pos.CopyFrom(pos)
        planned = egm_pb2.EgmPlanned()
        planned.cartesian.CopyFrom(pose)
        msg = egm_pb2.EgmSensor()
        msg.header.CopyFrom(header)
        msg.planned.CopyFrom(planned)

        self.debug_command_pose(pos, orient)

        sent = self.sock.sendto(msg.SerializeToString(), self.addr)

    def set_robot_velocity(self, vel, hz):
        # Position is changed according to vel
        # Orientation is given by original orientation
        pose = PoseStamped()
        pose.pose.position.x = self.lastfb.cartesian.pos.x + vel.pose.position.x/hz
        pose.pose.position.y = self.lastfb.cartesian.pos.y + vel.pose.position.y/hz
        pose.pose.position.z = self.lastfb.cartesian.pos.z + vel.pose.position.z/hz
        pose.pose.orientation.w = self.lastfb.cartesian.orient.u0
        pose.pose.orientation.x = self.lastfb.cartesian.orient.u1
        pose.pose.orientation.y = self.lastfb.cartesian.orient.u2
        pose.pose.orientation.z = self.lastfb.cartesian.orient.u3
        self.set_robot_pose(pose)

    # Debug functions
    def debug_measured_pose(self, fb):
        rospy.loginfo('[EGMControl] Measured pos (x, y, z) = (%.2f, %.2f, %.2f) mm', fb.cartesian.pos.x, fb.cartesian.pos.y, fb.cartesian.pos.z)
        rospy.loginfo('[EGMControl] Measured orient (w, x, y, z) = (%.2f, %.2f, %.2f, %.2f)', fb.cartesian.orient.u0, fb.cartesian.orient.u1, fb.cartesian.orient.u2, fb.cartesian.orient.u3)

    def debug_command_pose(self, pos, orient):
        rospy.loginfo('[EGMControl] Command pos (x, y, z) = (%.2f, %.2f, %.2f) mm', pos.x, pos.y, pos.z)
        rospy.loginfo('[EGMControl] Command orient (w, x, y, z) = (%.2f, %.2f, %.2f, %.2f)', orient.u0, orient.u1, orient.u2, orient.u3)
