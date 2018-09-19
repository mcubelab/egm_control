#!/usr/bin/env python
import time
import socket
import egm_pb2, egm_helper
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, Float32MultiArray
from ik.helper import quat_from_yaw, qwxyz_from_qxyzw, transform_back
import numpy as np
import tf


def GetTickCount():
    return int((time.time() + 0.5) * 1000)

class EGMController():

    def __init__(self, listener):
        self.UDP_PORT=6510
        self.sequenceNumber = 0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("", self.UDP_PORT))
        self.starttick = egm_helper.GetTickCount()
        data, self.addr = self.sock.recvfrom(1024)
        self.joint_pub = rospy.Publisher("/joint_states", JointState, queue_size = 2)
        self.cart_sensed_pub = rospy.Publisher("/cart_sensed_states", Float32MultiArray, queue_size = 2)
        self.cart_command_pub = rospy.Publisher("/cart_command_states", Float32MultiArray, queue_size = 2)
        self.listener = listener

    def get_robot_pos(self):
        try:
            egm_robot = egm_pb2.EgmRobot()
            data, addr = self.sock.recvfrom(1024)
            egm_robot.ParseFromString(data)
            pos_read = egm_robot.feedBack.cartesian.pos
            orient_read = egm_robot.feedBack.cartesian.orient
            pos_read = pos_read.x,  pos_read.y,  pos_read.z
            # publish robot joints (visualize rviz)
            self.publish_robot_joints(egm_robot.feedBack.joints.joints)
            # (trans, rot) = self.listener.lookupTransform("map", "/track_start", rospy.Time(0))
            self.publish_robot_cart([egm_robot.feedBack.cartesian.pos.x/1000. - 0.35, egm_robot.feedBack.cartesian.pos.y/1000.], self.cart_sensed_pub)
            return
        except Exception as e:
            return None

    def publish_robot_joints(self, joints):
        js = JointState()
        js.header = Header()
        js.header.stamp = rospy.Time.now()
        js.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        js.position = [j for j in joints]
        js.velocity = [0.0 for i in xrange(6)]
        js.effort = [0.0 for i in xrange(6)]
        self.joint_pub.publish(js)

    def publish_robot_cart(self, pos, pub):
        pos_xy = np.array(pos[0:2])
        js = Float32MultiArray()
        js.data = np.array(pos_xy)
        pub.publish(js)

    def send_robot_pos(self, position, theta=0):
        #convert position to robot frame (map)
        # import pdb;pdb.set_trace()
        (trans, rot) = self.listener.lookupTransform("map", "/track_start", rospy.Time(0))
        # import pdb;pdb.set_trace()
        # print 'position', position
        position = position + trans
        # print 'trans', trans
        # print 'position', position
        #apply workspace limits for safety
        x_limits = [0.1, 0.6]
        y_limits = [-0.4, 0.4]
        z_limits = [0, 0.5]
        position[0] = max(min(position[0], x_limits[1]), x_limits[0])
        position[1] = max(min(position[1], y_limits[1]), y_limits[0])
        position[2] = max(min(position[2], z_limits[1]), z_limits[0])
        #convert to mm
        position = position*1000
        # print 'position', position
        #Get header info
        egm_sensor_write = egm_pb2.EgmSensor()
        header = egm_pb2.EgmHeader()
        header.mtype = egm_pb2.EgmHeader.MSGTYPE_CORRECTION
        header.seqno = self.sequenceNumber
        self.sequenceNumber += 1
        header.tm = egm_helper.GetTickCount()-self.starttick
        egm_sensor_write.header.CopyFrom(header)
        #build robot pose object
        pos = egm_pb2.EgmCartesian()
        pos.x, pos.y, pos.z = position[0], position[1], position[2]
        orient = egm_pb2.EgmQuaternion()
        q = transform_back([0,0,0,0,1,0,0], [0,0,0]+quat_from_yaw(theta))[3:7]
        qtuple = tuple(qwxyz_from_qxyzw(q))
        orient.u0, orient.u1, orient.u2, orient.u3 = qtuple
        pose = egm_pb2.EgmPose()
        pose.orient.CopyFrom(orient)
        pose.pos.CopyFrom(pos)
        #build pose plan object
        planned = egm_pb2.EgmPlanned()
        planned.cartesian.CopyFrom(pose)
        egm_sensor_write.planned.CopyFrom(planned)
        #send robot command message
        sent = self.sock.sendto(egm_sensor_write.SerializeToString(),self.addr)

    def send_robot_vel(self, vel, h):
        #Get new position from velocity
        # h = 1./rate
        #convert velocity to 3d array
        vel = np.array(vel)
        vel = np.append(vel, 0.)
        #update position
        self.position += vel*h
        # print 'vel', vel
        # print 'position', self.position
        #send position command to robot
        self.send_robot_pos(self.position)
        #publish position command
        self.publish_robot_cart(self.position, self.cart_command_pub)
        return
