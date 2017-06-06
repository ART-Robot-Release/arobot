#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

import numpy as np

global js
js = JointState()

def record(cmd):
    joint_names = [
            'waist_pitch',
            'waist_yaw',
            'l_hip_yaw',
            'l_hip_roll',
            'l_hip_pitch',
            'l_knee_pitch',
            'l_ankle_roll',
            'l_ankle_pitch',
            'r_hip_yaw',
            'r_hip_roll',
            'r_hip_pitch',
            'r_knee_pitch',
            'r_ankle_roll',
            'r_ankle_pitch',
            'l_shoulder_pitch',
            'l_shoulder_roll',
            'l_elbow_yaw',
            'l_elbow_pitch',
            'l_wrist_yaw',
            'r_shoulder_pitch',
            'r_shoulder_roll',
            'r_elbow_yaw',
            'r_elbow_pitch',
            'r_wrist_yaw',
            'neck_yaw',
            'neck_pitch'
            ]


    global js
    # print ("Record the js data")
    point_pos_str = ""
    for i in joint_names:
        point_pos_str += ", %.02f" % js.position[js.name.index(i)]

    print ("time +=tparam")
    print ("point= copy.deepcopy(point_start)")
    print ("self.move_all(point, time %s)" % point_pos_str)
    print ("msg.points.append(point)\n")

def echo(data):
    global js
    js = data
# print ("The data of joint is :\n")
# print (js)


def listener():
    rospy.init_node('control_demo_echo', anonymous = False)
    rospy.Subscriber("/arobot/joint_states", JointState, echo)
    rospy.Subscriber("/arobot/record_cmd", String, record)

    rospy.spin()


if __name__ == '__main__':
    x = 0
    listener()

