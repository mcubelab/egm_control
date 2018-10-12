#!/usr/bin/env python
import os, sys
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

initial_pos = [300, 0, 178.2]
tol = 5.0 # mm tolerance for continuity between functions

def f1(t):
    return [
        initial_pos[0] + 4.0*t,
        initial_pos[1],
        initial_pos[2]
    ]

def f2(t):
    return [
        initial_pos[0] + 20.0*np.cos(t/2),
        initial_pos[1] + 20.0*np.sin(t/2),
        initial_pos[2]
    ]

def f3(t):
    return [
        initial_pos[0] + 20.0 - 4.0*t,
        initial_pos[1] + 4.0*t,
        initial_pos[2]
    ]

def f4(t):
    return [
        initial_pos[0] - 4.0*t,
        initial_pos[1] + 20.0 - 4.0*t,
        initial_pos[2]
    ]

def f5(t):
    return [
        initial_pos[0] - 20.0 + 4.0*t,
        initial_pos[1] - 4.0*t,
        initial_pos[2]
    ]

def f6(t):
    return [
        initial_pos[0] + 4.0*t,
        initial_pos[1] - 20.0 + 4.0*t,
        initial_pos[2]
    ]

# All sizes in mm!
hz = 248.0 # hz
config = [
    # [vx, vy, vz, t]
    [f1, 5.0],
    [f2, 8*np.pi],
    [f3, 5.0],
    [f4, 5.0],
    [f5, 5.0],
    [f6, 5.0],
    [f3, 5.0],
    [f4, 5.0],
    [f5, 5.0],
    [f6, 5.0],
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
            print(i)
            start_time = rospy.Time.now().to_sec()

        rate.sleep()
