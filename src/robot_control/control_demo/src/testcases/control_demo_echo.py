#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
import numpy as np


def echo(data):
    print ("The data of joint is :\n")
    print (data)

def listener():
    rospy.init_node('control_demo_echo', anonymous = False)
    rospy.Subscriber("/joint_states", JointState, echo)

    # spin() keeps python from exiting until
    rospy.spin()


if __name__ == '__main__':
    x = 0
    listener()
