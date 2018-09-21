#!/usr/bin/env python
import time
import rospy
import socket
import helpers.egm_pb2 as egm_pb2
import helpers.egm_helper as egm_helper

class RobotController():

    def __init__(self, pos, x_limits, y_limits, z_limits):
        self.UDP_PORT = 6510
        self.sequenceNumber = 0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("", self.UDP_PORT))
        data, self.addr = self.sock.recvfrom(1024)
        self.starttick = egm_helper.get_tick()
        self.pos = pos
        self.x_limits = x_limits
        self.y_limits = y_limits
        self.z_limits = z_limits

    def __del__(self):
        self.sock.close()

    def get_robot_feedback(self):
        try:
            egm_robot = egm_pb2.EgmRobot()
            data, addr = self.sock.recvfrom(1024)
            egm_robot.ParseFromString(data)
            fb = egm_robot.feedBack
            # print("Measured pose:")
            # print(fb.cartesian.pos.x)
            # print(fb.cartesian.pos.y)
            # print(fb.cartesian.pos.z)
            # print(fb.cartesian.orient.u0)
            # print(fb.cartesian.orient.u1)
            # print(fb.cartesian.orient.u2)
            # print(fb.cartesian.orient.u3)
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
        pos.x = max(min(pose.pose.position.x, self.x_limits[1]), self.x_limits[0])*1000
        pos.y = max(min(pose.pose.position.y, self.y_limits[1]), self.y_limits[0])*1000
        pos.z = max(min(pose.pose.position.z, self.z_limits[1]), self.z_limits[0])*1000

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

        # Updating self.pos
        self.pos[0] = pos.x
        self.pos[1] = pos.y
        self.pos[2] = pos.z

        # print("Command pose:")
        # print(pos.x)
        # print(pos.y)
        # print(pos.z)
        # print(orient.u0)
        # print(orient.u1)
        # print(orient.u2)
        # print(orient.u3)

        sent = self.sock.sendto(msg.SerializeToString(), self.addr)

    def set_robot_velocity(self, vel, hz):
        # Position is changed according to vel
        # Orientation is given by original orientation
        pose = vel
        pose.pose.position.x = self.pos[0] + vel.pose.position.x/hz
        pose.pose.position.y = self.pos[1] + vel.pose.position.y/hz
        pose.pose.position.z = self.pos[2] + vel.pose.position.z/hz
        self.set_robot_pose(pose)
