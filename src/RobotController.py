#!/usr/bin/env python
import time
import rospy
import socket
import helpers.egm_pb2 as egm_pb2
import helpers.EGMHelper as EGMHelper
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Header

class RobotController():

    def __init__(self, x_limits, y_limits, z_limits):
        # Initialize connection
        self.UDP_PORT = 6510
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("", self.UDP_PORT))
        data, self.addr = self.sock.recvfrom(1024)

        # Initialize variables
        self.last_measured_pose = EGMHelper.EgmFeedback_to_PoseStamped(self.get_feedback_from_data(data))
        self.last_command_time = rospy.Time()   # Stamp of last command_pose received
        self.last_sent_pose = self.last_measured_pose

        self.sequenceNumber = 0
        self.start_tick = EGMHelper.get_tick()
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
        data, addr = self.sock.recvfrom(1024)
        feedback = self.get_feedback_from_data(data)
        self.last_measured_pose = EGMHelper.EgmFeedback_to_PoseStamped(feedback)
        joint_state = EGMHelper.EgmFeedback_to_JointState(feedback)
        return self.last_measured_pose, joint_state

    def send_command(self, command_pose, command_mode, hz):
        # Check if last received command has already been executed
        # In this case, default behavior - stay at same position
        if command_pose.header.stamp == self.last_command_time:
            target = self.last_measured_pose.pose
        else:
            if command_mode == 'velocity':
                if self.last_command_time == rospy.Time():
                    dt = 1/hz
                else:
                    dt = 1/hz
                    # TODO: improve this
                if rospy.get_param('egm_vel_mode', 'virtual') == 'real':
                    target = EGMHelper.translate_pose_by_velocity(self.last_measured_pose.pose, command_pose.pose, dt)
                else:
                    target = EGMHelper.translate_pose_by_velocity(self.last_sent_pose.pose, command_pose.pose, dt)
            else:
                target = command_pose.pose

        # Apply limits to the target position
        target.position.x = max(min(target.position.x, self.x_limits[1]), self.x_limits[0])
        target.position.y = max(min(target.position.y, self.y_limits[1]), self.y_limits[0])
        target.position.z = max(min(target.position.z, self.z_limits[1]), self.z_limits[0])

        msg = EGMHelper.Pose_to_EgmSensor(target, self.sequenceNumber, EGMHelper.get_tick()-self.start_tick)
        self.sequenceNumber += 1
        self.last_sent_pose = PoseStamped(header=Header(stamp=rospy.Time.now(), frame_id='map'), pose=target)
        self.last_command_time = command_pose.header.stamp
        sent = self.sock.sendto(msg.SerializeToString(), self.addr)
        return self.last_sent_pose

    # Debug functions
    def debug_connection(self, addr, fb):
        rospy.loginfo('[EGMControl] Successfully connected to robot. IP: %s', addr)
        rospy.loginfo('[EGMControl] Initial position (x, y, z) = (%.2f, %.2f, %.2f) mm', fb.cartesian.pos.x, fb.cartesian.pos.y, fb.cartesian.pos.z)
        rospy.loginfo('[EGMControl] Initial position (w, x, y, z) = (%d, %d, %d, %d)', fb.cartesian.orient.u0, fb.cartesian.orient.u1, fb.cartesian.orient.u2, fb.cartesian.orient.u3)

    def debug_measured_pose(self, fb):
        rospy.loginfo('[EGMControl] Measured pos (x, y, z) = (%.2f, %.2f, %.2f) mm', fb.cartesian.pos.x, fb.cartesian.pos.y, fb.cartesian.pos.z)
        rospy.loginfo('[EGMControl] Measured orient (w, x, y, z) = (%.2f, %.2f, %.2f, %.2f)', fb.cartesian.orient.u0, fb.cartesian.orient.u1, fb.cartesian.orient.u2, fb.cartesian.orient.u3)

    def debug_command_pose(self, pos, orient):
        rospy.loginfo('[EGMControl] Command pos (x, y, z) = (%.2f, %.2f, %.2f) mm', pos.x, pos.y, pos.z)
        rospy.loginfo('[EGMControl] Command orient (w, x, y, z) = (%.2f, %.2f, %.2f, %.2f)', orient.u0, orient.u1, orient.u2, orient.u3)
