#!/usr/bin/env python
# -* - coding: UTF-8 -* -
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import std_msgs.msg
import copy

def angleToRad(angle):
    return angle*3.1416/180

class robotDemo:
    joint_names = []
    joint_ids = {}

    def __init__(self):
        self.joint_names = [
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

        for i, name in enumerate(self.joint_names):
            self.joint_ids[name] = i

    def set_the_rad(self, point, angle, name):
        point.positions[self.joint_ids[name]] = angleToRad(angle)

    def get_msg_with_name(self):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        return msg

    def move_to_start(self):
        msg = self.get_msg_with_name()
        point_start = JointTrajectoryPoint()
        point_start.positions = [0 for i in range(26)]
        point_start.time_from_start = rospy.Duration(3)
        msg.points.append(point_start)

        return msg


    def wave_hand(self):
        msg = self.get_msg_with_name()
        point_start = JointTrajectoryPoint()
        point_start.positions = [0 for i in range(26)]
        point_start.time_from_start = rospy.Duration(1)
        msg.points.append(point_start)

        # point1 = JointTrajectoryPoint()
        point1 = copy.deepcopy(point_start)
        point1.positions = [0 for i in range(26)]
        self.set_the_rad(point1, -80, 'r_shoulder_pitch')
        self.set_the_rad(point1, -90, 'r_elbow_pitch')

        point1.time_from_start = rospy.Duration(5)
        msg.points.append(point1)

        point2 = copy.deepcopy(point1)
        point2.time_from_start = rospy.Duration(6)
        self.set_the_rad(point2, -30, 'r_elbow_yaw')
        self.set_the_rad(point2, -15,'r_wrist_yaw')
        self.set_the_rad(point2, -15, 'neck_pitch')
        self.set_the_rad(point2, -15, 'neck_yaw')
        msg.points.append(point2)

        point3 = copy.deepcopy(point1)
        point3.time_from_start = rospy.Duration(7)
        self.set_the_rad(point3, 30, 'r_elbow_yaw')
        self.set_the_rad(point3, 15, 'r_wrist_yaw')
        self.set_the_rad(point3, 10, 'neck_pitch')
        self.set_the_rad(point3, 15, 'neck_yaw')
        msg.points.append(point3)

        point4 = copy.deepcopy(point1)
        point4.time_from_start = rospy.Duration(9)
        self.set_the_rad(point4, -30, 'r_elbow_yaw')
        self.set_the_rad(point4, -15, 'r_wrist_yaw')
        self.set_the_rad(point4, -15, 'neck_pitch')
        self.set_the_rad(point4, -15, 'neck_yaw')
        msg.points.append(point4)

        point5 = copy.deepcopy(point1)
        point5.time_from_start = rospy.Duration(11)
        self.set_the_rad(point5, 30, 'r_elbow_yaw')
        self.set_the_rad(point5, 15, 'r_wrist_yaw')
        self.set_the_rad(point5, 10, 'neck_pitch')
        self.set_the_rad(point5, 15, 'neck_yaw')
        msg.points.append(point5)

        point6 = copy.deepcopy(point1)
        point6.time_from_start = rospy.Duration(12)
        msg.points.append(point6)

        point7 = copy.deepcopy(point_start)
        point7.time_from_start = rospy.Duration(15)
        msg.points.append(point7)

        return msg

if __name__ == '__main__':

    pub = rospy.Publisher('/arobot/arobot_trajectory_controller/command', JointTrajectory,queue_size=1000)

    rospy.init_node('control_demo_trajectory_pub', anonymous = False)

    demo = robotDemo()

    rospy.sleep(3)

    pub.publish(demo.wave_hand())
