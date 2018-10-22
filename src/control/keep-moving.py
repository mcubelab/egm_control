#!/usr/bin/env python
import os, sys
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

initial_pos = [0.3, 0.0, 0.1782]
tol = 0.005 # m tolerance for continuity between functions

def f0(t):
    return initial_pos

def f1(t):
    return [
        initial_pos[0] + 0.004*t,
        initial_pos[1],
        initial_pos[2]
    ]

def f2(t):
    return [
        initial_pos[0] + 0.02*np.cos(t/2),
        initial_pos[1] + 0.02*np.sin(t/2),
        initial_pos[2]
    ]

# All sizes in mm!
hz = 248.0 # hz
config = [
    # [vx, vy, vz, t]
    [f0, 5.0],
    [f1, 5.0],
    [f2, 1.0e9],
]

if __name__ == '__main__':
    rospy.init_node('exampleblock', anonymous=True)
    command_pose_pub = rospy.Publisher('/command_pose', PoseStamped, queue_size = 100, latch=True)

    rospy.set_param('egm_mode', 'position')

    rate = rospy.Rate(hz)
    start_time = rospy.Time.now().to_sec()
    i = 0

    while (not rospy.is_shutdown()) and i < len(config):
        now = rospy.Time.now()
        t = now.to_sec()-start_time
        pose = PoseStamped()
        pose.header = Header()
        pose.header.stamp = now
        pose.header.frame_id = "map"
        # Position in mm or velocity in mm/s
        pos = config[i][0](t)
        pose.pose.position.x = pos[0]
        pose.pose.position.y = pos[1]
        pose.pose.position.z = pos[2]
        # Orientation or angular velocity in xyzw
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = -1.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 0.0
        command_pose_pub.publish(pose)

        if rospy.Time.now().to_sec()-start_time >= config[i][1]:
            # print(str(i) + ': last pose is ' + str(pos[0]) + ', '+ str(pos[1]) + ', '+ str(pos[2]))
            if i+1 < len(config):
                pos2 = config[i+1][0](0)
                # print(str(i+1) + ': first pose will be ' + str(pos2[0]) + ', '+ str(pos2[1]) + ', '+ str(pos2[2]) )
                if abs(pos2[0]-pos[0]) > tol or abs(pos2[1]-pos[1]) > tol or abs(pos2[2]-pos[2]) > tol:
                    print('ERROR: Position continuity is not held between orders ' + str(i) + ' and ' + str(i+1) + ' (starting from 0).')
                    print('Command pose sending stopped to avoid high dynamic load in robot.')
                    break
            i += 1
            start_time = rospy.Time.now().to_sec()

        rate.sleep()
