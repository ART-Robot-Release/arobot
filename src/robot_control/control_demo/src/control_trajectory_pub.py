#!/usr/bin/env python
# -* - coding: UTF-8 -* -
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import std_msgs.msg
import numpy as np
import random
import copy
import sys, getopt

def angleToRad(angle):
    return angle*3.1416/180

class robotDemo:
    joint_names = []
    joint_ids = {}
    ros_2_cal = []

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

        self.ros_2_cal = rospy.get_param(rospy.search_param('ros_2_cal'));

        print(self.ros_2_cal)


    def get_the_rad(self, angle, name):
        return  angleToRad(angle) * self.ros_2_cal[name]

    def set_the_rad(self, point, angle, name):
        point.positions[self.joint_ids[name]] = self.get_the_rad(angle, name)

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

    def move_two_arm(self):
        msg = self.get_msg_with_name()
        point_start = JointTrajectoryPoint()
        point_start.positions = [0 for i in range(26)]
        point_start.time_from_start = rospy.Duration(1)
        msg.points.append(point_start)

        point1 = JointTrajectoryPoint()
        point1.positions = [0 for i in range(26)]
        self.set_the_rad(point1, 30, 'l_shoulder_pitch')
        self.set_the_rad(point1, -30, 'r_shoulder_pitch')
        self.set_the_rad(point1, 10, 'neck_yaw')

        point1.time_from_start = rospy.Duration(2)
        msg.points.append(point1)

        point2 = copy.deepcopy(point1)
        point2.time_from_start = rospy.Duration(4)
        self.set_the_rad(point2, -30, 'l_shoulder_pitch')
        self.set_the_rad(point2, 30, 'r_shoulder_pitch')
        self.set_the_rad(point2, -10, 'neck_yaw')
        msg.points.append(point2)

        point3 = copy.deepcopy(point_start)
        point3.time_from_start = rospy.Duration(5)
        msg.points.append(point3)

        return msg

    def throw(self):
        tparam = 3;
        msg = self.get_msg_with_name()
        point_start = JointTrajectoryPoint()
        point_start.positions = [0 for i in range(26)]
        point_start.time_from_start = rospy.Duration(1 * tparam)
        msg.points.append(point_start)

        point1 = copy.deepcopy(point_start)
        self.set_the_rad(point1, -90,'r_shoulder_pitch')
        self.set_the_rad(point1, -0,'r_shoulder_roll')
        self.set_the_rad(point1, -110,'r_elbow_pitch')
        self.set_the_rad(point1, 0,'r_elbow_yaw')
        self.set_the_rad(point1, -90,'r_wrist_yaw')
        self.set_the_rad(point1, -90,'l_shoulder_pitch')
        self.set_the_rad(point1, -10,'l_shoulder_roll')
        self.set_the_rad(point1, -45,'l_elbow_pitch')
        self.set_the_rad(point1, -45,'l_elbow_yaw')
        self.set_the_rad(point1, -90,'l_wrist_yaw')
        self.set_the_rad(point1, -30,'l_hip_pitch')
        self.set_the_rad(point1, 60, 'l_knee_pitch')
        self.set_the_rad(point1, -30,'l_ankle_pitch')
        self.set_the_rad(point1, -30,'r_hip_pitch')
        self.set_the_rad(point1, 60, 'r_knee_pitch')
        self.set_the_rad(point1, -30,'r_ankle_pitch')
        point1.time_from_start = rospy.Duration(2 * tparam)
        msg.points.append(point1)

        point2 = copy.deepcopy(point_start)
        self.set_the_rad(point2, -130,'r_shoulder_pitch')
        self.set_the_rad(point2, 0,'r_shoulder_roll')
        self.set_the_rad(point2, 0,'r_elbow_pitch')
        self.set_the_rad(point2, 0,'r_elbow_yaw')
        self.set_the_rad(point2, 0,'r_wrist_yaw')

        self.set_the_rad(point2, -100,'l_shoulder_pitch')
        self.set_the_rad(point2, 0,'l_shoulder_roll')
        self.set_the_rad(point2, 0,'l_elbow_pitch')
        self.set_the_rad(point2, 0,'l_elbow_yaw')
        self.set_the_rad(point2, 0,'l_wrist_yaw')
        point2.time_from_start = rospy.Duration(2.5 * tparam)
        msg.points.append(point2)

        point3 = copy.deepcopy(point_start)
        point3.time_from_start = rospy.Duration(4 * tparam)
        msg.points.append(point3)
        return msg

    def move_one(self,point,angle0,angle1,angle2,angle3,angle4,angle5,angle6,angle7,angle8,angle9,Durationtime):

        self.set_the_rad(point,angle0, 'l_shoulder_pitch')
        self.set_the_rad(point,angle1, 'l_shoulder_roll')
        self.set_the_rad(point,angle2, 'l_elbow_yaw')
        self.set_the_rad(point,angle3, 'l_elbow_pitch')
        self.set_the_rad(point,angle4, 'l_wrist_yaw')

        self.set_the_rad(point, angle5, 'r_shoulder_pitch')
        self.set_the_rad(point, angle6, 'r_shoulder_roll')
        self.set_the_rad(point, angle7, 'r_elbow_yaw')
        self.set_the_rad(point, angle8, 'r_elbow_pitch')
        self.set_the_rad(point, angle9, 'r_wrist_yaw')
        point.time_from_start = rospy.Duration(Durationtime)

    def down_leg(self,point,angle):

        self.set_the_rad(point,-1.0*angle, 'l_hip_pitch')
        self.set_the_rad(point,angle*2.0, 'l_knee_pitch')
        self.set_the_rad(point,-1.0*angle, 'l_ankle_pitch')

        self.set_the_rad(point,-1.0*angle, 'r_hip_pitch')
        self.set_the_rad(point,angle*2.0, 'r_knee_pitch')
        self.set_the_rad(point,-1.0*angle, 'r_ankle_pitch')

    def tilt_leg(self,point,angle):

        self.set_the_rad(point,1.0*angle, 'l_hip_roll')
        self.set_the_rad(point,-1.0*angle, 'l_ankle_roll')

        self.set_the_rad(point,1.0*angle, 'r_hip_roll')
        self.set_the_rad(point,-1.0*angle, 'r_ankle_roll')

    def move_all(self,point, Durationtime, *args):

        for i,ele in enumerate(args):
            point.positions[i] = float(ele)

        point.time_from_start = rospy.Duration(Durationtime)


    def test(self, tparam):
        time = 1
        msg = self.get_msg_with_name()
        point_start = JointTrajectoryPoint()
        point_start.positions = [0 for i in range(26)]
        point_start.time_from_start = rospy.Duration(time)
        msg.points.append(point_start)

        time +=tparam
        point= copy.deepcopy(point_start)
        self.move_all(point, time , -0.00, 0.00, -0.00, 0.08, -0.09, 0.02, -0.00, -0.11, -0.00, 0.08, -0.04, -0.01, -0.26, -0.00, -0.74, 0.00, -0.90, -1.35, -0.00, -0.41, -0.03, -0.27, -1.12, 0.00, 0.00, 0.00)
        msg.points.append(point)

        time +=tparam
        point= copy.deepcopy(point_start)
        self.move_all(point, time , 0.01, -0.00, -0.00, 0.06, -0.09, 0.02, -0.00, -0.11, -0.00, 0.09, -0.04, -0.01, -0.26, -0.00, -0.35, 0.00, -0.52, -0.79, -0.00, -0.07, -1.05, -0.79, -1.00, 0.00, 0.00, 0.00)
        msg.points.append(point)

        return msg
    def hand(self,tparam):
        time = 1
        msg = self.get_msg_with_name()
        point_start = JointTrajectoryPoint()
        point_start.positions = [0 for i in range(26)]
        point_start.time_from_start = rospy.Duration(time)
        msg.points.append(point_start)

        time += tparam
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, -45, 0, 0, -60, 0, time)
        msg.points.append(point) 

        time += tparam
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, -45, 0, 0, -60, 0, time)
        msg.points.append(point) 

        time += tparam
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, -30, 0, 0, -45, 0, time)
        msg.points.append(point) 

        time += tparam
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, -45, 0, 0, -60, 0, time)
        msg.points.append(point) 

        time += tparam
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, time)
        msg.points.append(point) 

        return msg
    def ceshi(self,tparam):
        time = 1
        msg = self.get_msg_with_name()
        point_start = JointTrajectoryPoint()
        point_start.positions = [0 for i in range(26)]
        point_start.time_from_start = rospy.Duration(time)
        msg.points.append(point_start)
        # 第3个八拍动作开始
        time += tparam 
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 45, 0, 0, 0, 0, 0, 0, 0, 0, time)
        msg.points.append(point)
        time += tparam
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, time)
        msg.points.append(point)
        return msg
    def dance7(self,tparam):
        time = 1
        msg = self.get_msg_with_name()
        point_start = JointTrajectoryPoint()
        point_start.positions = [0 for i in range(26)]
        self.down_leg(point_start,10)
        point_start.time_from_start = rospy.Duration(time)
        msg.points.append(point_start)

        time +=tparam*2 
        point= copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 0, 0, 90, -90, 0, 0, 0, -90, time)
        msg.points.append(point)
        # 第9个八拍动作开始
        time +=tparam  # 第1拍
        point= copy.deepcopy(point_start)
        self.move_one(point, -90, 0, -30, -90, 90, -90, 0, -30, -90, -90, time)
        self.tilt_leg(point,5)
        msg.points.append(point)
        time +=tparam  # 第2拍
        point= copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 30, -90, 90, -90, 0, 30, -90, -90, time)
        self.tilt_leg(point,-5)
        msg.points.append(point)
        time +=tparam # 第3拍
        point= copy.deepcopy(point_start)
        self.move_one(point, -90, 0, -30, -90, 90, -90, 0, -30, -90, -90, time)
        self.tilt_leg(point,5)
        msg.points.append(point)
        time +=tparam # 第4拍
        point= copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 30, -90, 90, -90, 0, 30, -90, -90, time)
        self.tilt_leg(point,-5)
        msg.points.append(point)
        time +=tparam # 第5拍
        point= copy.deepcopy(point_start)
        self.move_one(point, -90, 0, -30, -90, 90, -90, 0, -30, -90, -90, time)
        self.tilt_leg(point,5)
        msg.points.append(point)
        time +=tparam # 第6拍
        point= copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 30, -90, 90, -90, 0, 30, -90, -90, time)
        self.tilt_leg(point,-5)
        msg.points.append(point)
        time +=tparam # 第7拍
        point= copy.deepcopy(point_start)
        self.move_one(point, -90, 0, -30, -90, 90, -90, 0, -30, -90, -90, time)
        self.tilt_leg(point,5)
        msg.points.append(point)
        time +=tparam # 第8拍
        point= copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 30, -90, 90, -90, 0, 30, -90, -90, time)
        self.tilt_leg(point,-5)
        msg.points.append(point)
        # 第9个八拍动作结束
        time +=tparam*3 
        point= copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, time)
        self.tilt_leg(point,0)
        self.down_leg(point,0)
        msg.points.append(point)
        return msg
    def dance0(self,tparam):
        time = 1
        msg = self.get_msg_with_name()
        point_start = JointTrajectoryPoint()
        point_start.positions = [0 for i in range(26)]
        self.down_leg(point_start,10)
        point_start.time_from_start = rospy.Duration(time)
        msg.points.append(point_start)
        # 第1个八拍动作开始
        time +=tparam*0.7   # 第1拍
        point= copy.deepcopy(point_start)
        self.down_leg(point,10)
        self.move_one(point, -90, 0, 0, 0, 0, 0, 0, 0, 0, 0, time)
        msg.points.append(point)
        time +=tparam*0.3
        point= copy.deepcopy(point_start)
        self.down_leg(point,10)
        self.move_one(point, -90, 0, 0, 0, 0, 0, 0, 0, 0, 0, time)
        msg.points.append(point)

        time += tparam*0.7    # 第2拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 0, -90, 0, 0, 0, 0, 0, 0, time)
        self.down_leg(point,10)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 0, -90, 0, 0, 0, 0, 0, 0, time)
        self.down_leg(point,10)
        msg.points.append(point)

        time += tparam*0.7   # 第3拍
        point= copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 0, 0, 0, 0, 0, 0, 0, 0, time)
        self.down_leg(point,10)
        msg.points.append(point)
        time += tparam*0.3
        point= copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 0, 0, 0, 0, 0, 0, 0, 0, time)
        self.down_leg(point,10)
        msg.points.append(point)

        time += tparam*0.7   # 第4拍
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, time)
        self.down_leg(point,15)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, time)
        self.down_leg(point,15)
        msg.points.append(point)

        time += tparam*0.7   # 第5拍
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, -90, 0, 0, 0, 0, time)
        self.down_leg(point,10)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, -90, 0, 0, 0, 0, time)
        self.down_leg(point,10)
        msg.points.append(point)

        time += tparam*0.7   # 第6拍
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, -90, 0, 0, -90, 0, time)
        self.down_leg(point,10)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, -90, 0, 0, -90, 0, time)
        self.down_leg(point,10)
        msg.points.append(point)

        time += tparam*0.7   # 第7拍
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, -90, 0, 0, 0, 0, time)
        self.down_leg(point,10)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, -90, 0, 0, 0, 0, time)
        self.down_leg(point,10)
        msg.points.append(point)

        time += tparam*0.7   # 第8拍
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, time)
        self.down_leg(point,15)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, time)
        self.down_leg(point,15)
        msg.points.append(point)
        # 第1个八拍动作结束
        time += tparam
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, time)
        self.down_leg(point,0)
        msg.points.append(point)

        return msg
    def dance1(self,tparam):
        time = 1
        msg = self.get_msg_with_name()
        point_start = JointTrajectoryPoint()
        point_start.positions = [0 for i in range(26)]
        self.down_leg(point_start,10)
        point_start.time_from_start = rospy.Duration(time)
        msg.points.append(point_start)

        time += tparam
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, time)
        self.down_leg(point,15)
        msg.points.append(point)
        # 第2个八拍动作开始
        time +=tparam*0.7   # 第1拍
        point= copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 0, 0, 0, -90, 0, 0, 0, 0, time)
        msg.points.append(point)
        time +=tparam*0.3
        point= copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 0, 0, 0, -90, 0, 0, 0, 0, time)
        msg.points.append(point)
        time += tparam*0.7   # 第2拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 0, 0, -90, -90, 0, 0, 0, 90, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 0, 0, -90, -90, 0, 0, 0, 90, time)
        msg.points.append(point)
        time += tparam*0.7   # 第3拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 90, -90, 0, -90, 0, 0, 0, 90, time)
        self.down_leg(point,15)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 90, -90, 0, -90, 0, 0, 0, 90, time)
        self.down_leg(point,15)
        msg.points.append(point)
        time += tparam*0.7   # 第4拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 90, -90, 0, -90, 0, 0, -90, 90, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 90, -90, 0, -90, 0, 0, -90, 90, time)
        msg.points.append(point)
        time += tparam/2*0.7   # 第5拍
        point = copy.deepcopy(point_start)
        self.set_the_rad(point, 45, 'neck_pitch')
        self.move_one(point, -90, 0, 90, -90, 0, -90, 0, 0, -90, 90, time)
        msg.points.append(point)
        time += tparam/2*0.7
        point = copy.deepcopy(point_start)
        self.set_the_rad(point, 0, 'neck_pitch')
        self.move_one(point, -90, 0, 90, -90, 0, -90, 0, 0, -90, 90, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.set_the_rad(point, 0, 'neck_pitch')
        self.move_one(point, -90, 0, 90, -90, 0, -90, 0, 0, -90, 90, time)
        msg.points.append(point)
        time += tparam/2*0.7   # 第6拍
        point = copy.deepcopy(point_start)
        self.set_the_rad(point, 45, 'neck_pitch')
        self.move_one(point, -90, 0, 90, -90, 0, -90, 0, 0, -90, 90, time)
        msg.points.append(point)
        time += tparam/2*0.7
        point = copy.deepcopy(point_start)
        self.set_the_rad(point, 0, 'neck_pitch')
        self.move_one(point, -90, 0, 90, -90, 0, -90, 0, 0, -90, 90, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.set_the_rad(point, 0, 'neck_pitch')
        self.move_one(point, -90, 0, 90, -90, 0, -90, 0, 0, -90, 90, time)
        msg.points.append(point)
        time += tparam*0.7   # 第7拍
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, time)
        self.down_leg(point,15)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, time)
        self.down_leg(point,15)
        msg.points.append(point)
        time += tparam*1   # 第8拍
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, time)
        msg.points.append(point)
        # 第2个八拍动作结束
        time += tparam
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, time)
        self.down_leg(point,0)
        msg.points.append(point)
        return msg

    def dance2(self,tparam):
        time = 1
        msg = self.get_msg_with_name()
        point_start = JointTrajectoryPoint()
        point_start.positions = [0 for i in range(26)]
        self.down_leg(point_start,10)
        point_start.time_from_start = rospy.Duration(time)
        msg.points.append(point_start)
        # 第3个八拍动作开始
        time += tparam*0.7   # 第1拍
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 90, 90, 0, -90, 0, 0, 0, 0, 0, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 90, 90, 0, -90, 0, 0, 0, 0, 0, time)
        msg.points.append(point)
        time += tparam*0.7   # 第2拍
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 90, 90, -90, 0, 0, 0, 0, 0, 0, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 90, 90, -90, 0, 0, 0, 0, 0, 0, time)
        msg.points.append(point)
        time += tparam*0.7   # 第3拍
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 90, 90, -90, 0, 0, -90, -90, 0, 90, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 90, 90, -90, 0, 0, -90, -90, 0, 90, time)
        msg.points.append(point)

        time += tparam*0.7   # 第4拍
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 90, 90, -90, 0, 0, -90, -90, -90, 0, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 90, 90, -90, 0, 0, -90, -90, -90, 0, time)
        msg.points.append(point)
        time += tparam/2*0.7   # 第5拍
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 90, 0, -90, 0, 0, -90, -90, -90, 0, time)
        msg.points.append(point)
        time += tparam/2*0.7
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 90, 90, -90, 0, 0, -90, -90, -90, 0, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 90, 90, -90, 0, 0, -90, -90, -90, 0,time)
        msg.points.append(point)
        time += tparam/2*0.7   # 第6拍
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 90, 0, -90, 0, 0, -90, -90, -90, 0,time)
        msg.points.append(point)
        time += tparam/2*0.7
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 90, 90, -90, 0,0, -90, -90, -90, 0, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 90, 90, -90, 0, 0, -90, -90, -90, 0,time)
        msg.points.append(point)
        time += tparam*0.7   # 第7拍
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 90, 90, 0, -90, 0, -90, -90, 0, 90, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 90, 90, 0, -90, 0, -90, -90, 0, 90, time)
        msg.points.append(point)
        time += tparam*0.7   # 第8拍
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, time)
        msg.points.append(point)

        # 第3个八拍动作结束
        return msg
    def dance_circle(self,tparam):
        time = 1.0
        msg = self.get_msg_with_name()
        point_start = JointTrajectoryPoint()
        point_start.positions = [0 for i in range(26)]
        self.down_leg(point_start,10)
        point_start.time_from_start = rospy.Duration(time)
        msg.points.append(point_start)
        # 第4个八拍动作开始
        time +=tparam*0.7   # 第1拍
        point= copy.deepcopy(point_start)
        self.move_one(point, -45, 0, 0, -90, 0,  -45, -30, 0, -90, 0, time)
        msg.points.append(point)
        time +=tparam*0.3
        point= copy.deepcopy(point_start)
        self.move_one(point, -45, 0, 0, -90, 0,  -45, -30, 0, -90, 0, time)
        msg.points.append(point)
        time += tparam*0.7   # 第2拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -60, 15, 0, -75, 0,  -60, -15, 0, -75, 0, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -60, 15, 0, -75, 0,  -60, -15, 0, -75, 0, time)
        msg.points.append(point)
        time += tparam*0.7   # 第3拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -45, 30, 0, -60, 0,  -45, 0, 0, -60, 0, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -45, 30, 0, -60, 0,  -45, 0, 0, -60, 0, time)
        msg.points.append(point)
        time += tparam*0.7   # 第4拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -30, 15, 0, -75, 0,  -30, -15, 0, -75, 0, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -30, 15, 0, -75, 0,  -30, -15, 0, -75, 0, time)
        msg.points.append(point)

        time += tparam*0.7   # 第5拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -45, 0, 0, -90, 0, -45, -30, 0, -90, 0, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -45, 0, 0, -90, 0, -45, -30, 0, -90, 0, time)
        msg.points.append(point)
        time += tparam*0.7   # 第6拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -60, 15, 0, -75, 0, -60, -15, 0, -75, 0, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -60, 15, 0, -75, 0, -60, -15, 0, -75, 0, time)
        msg.points.append(point)
        time += tparam*0.7   # 第7拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -45, 30, 0, -60, 0, -45, 0, 0, -60, 0, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -45, 30, 0, -60, 0, -45, 0, 0, -60, 0, time)
        msg.points.append(point)
        time += tparam*0.7   # 第8拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -30, 15, 0, -75, 0, -30, -15, 0, -75, 0, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -30, 15, 0, -75, 0, -30, -15, 0, -75, 0, time)
        msg.points.append(point)
        # 第4个八拍动作结束
        return msg

    def dance3(self,tparam):
        time = 1.0
        msg = self.get_msg_with_name()
        point_start = JointTrajectoryPoint()
        point_start.positions = [0 for i in range(26)]
        self.down_leg(point_start,10)
        point_start.time_from_start = rospy.Duration(time)
        msg.points.append(point_start)

        time += tparam
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, time)
        self.down_leg(point,15)
        msg.points.append(point)
        # 第5个八拍动作开始
        time += tparam*0.7# 第1拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -50, 10, 0, -90, 90, 0, 0, 0, 0, 0, time)
        # self.down_leg(point,10)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -50, 10, 0, -90, 90, 0, 0, 0, 0, 0, time)
        # self.down_leg(point,10)
        msg.points.append(point)
        time += tparam*0.7# 第2拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -50, 10, 0, -90, 90, -50, -10, 0, -90, -90, time)
        # self.down_leg(point,0)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -50, 10, 0, -90, 90, -50, -10, 0, -90, -90, time)
        # self.down_leg(point,0)
        msg.points.append(point)

        time += tparam/2*0.7# 第3拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -30, 10, 0, -90, 90, -50, -10, 0, -90, -90, time)
        msg.points.append(point)
        time += tparam/2*0.7
        point = copy.deepcopy(point_start)
        self.move_one(point, -50, 10, 0, -90, 90, -50, -10, 0, -90, -90, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -50, 10, 0, -90, 90, -50, -10, 0, -90, -90, time)
        msg.points.append(point)
        time += tparam/2*0.7# 第4拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -30, 10, 0, -90, 90, -50, -10, 0, -90, -90, time)
        msg.points.append(point)
        time += tparam/2*0.7
        point = copy.deepcopy(point_start)
        self.move_one(point, -50, 10, 0, -90, 90, -50, -10, 0, -90, -90, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -50, 10, 0, -90, 90, -50, -10, 0, -90, -90, time)
        msg.points.append(point)

        time += tparam/2*0.7# 第5拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -50, 10, 0, -90, 90, -30, -10, 0, -90, -90, time)
        msg.points.append(point)
        time += tparam/2*0.7
        point = copy.deepcopy(point_start)
        self.move_one(point, -50, 10, 0, -90, 90, -50, -10, 0, -90, -90, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -50, 10, 0, -90, 90, -50, -10, 0, -90, -90, time)
        msg.points.append(point)
        time += tparam/2*0.7# 第6拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -50, 10, 0, -90, 90, -30, -10, 0, -90, -90, time)
        msg.points.append(point)
        time += tparam/2*0.7
        point = copy.deepcopy(point_start)
        self.move_one(point, -50, 10, 0, -90, 90, -50, -10, 0, -90, -90, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -50, 10, 0, -90, 90, -50, -10, 0, -90, -90, time)
        msg.points.append(point)

        time += tparam*0.7# 第7拍
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, time)
        self.down_leg(point,15)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, time)
        self.down_leg(point,15)
        msg.points.append(point)
        time += tparam*0.7# 第8拍
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, time)
        msg.points.append(point)
        # 第5个八拍动作结束
        return msg
    def dance4(self,tparam):
        time = 1
        msg = self.get_msg_with_name()
        point_start = JointTrajectoryPoint()
        point_start.positions = [0 for i in range(26)]
        point_start.time_from_start = rospy.Duration(time)
        msg.points.append(point_start)
        # 第6个八拍动作开始
        time += tparam*0.7# 第1拍
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, -90, -90, -90, 0, 0, 0, 0, 0, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, -90, -90, -90, 0, 0, 0, 0, 0, time)
        msg.points.append(point)

        time += tparam*0.7# 第2拍
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, -90, 90, 0, 0, 0, 0, 0, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, -90, 90, 0, 0, 0, 0, 0, time)
        msg.points.append(point)

        time += tparam*0.7# 第3拍
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, -90, 90, 0, 0, 90, -90, 90, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, -90, 90, 0, 0, 90, -90, 90, time)
        msg.points.append(point)

        time += tparam*0.7# 第4拍
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, -90, 90, 0, 0, 0, -90, -90, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, -90, 90, 0, 0, 0, -90, -90, time)
        msg.points.append(point)

        time += tparam/2*0.7# 第5拍
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, -45, -90, 90, 0, 0, 0, -90, -90, time)
        msg.points.append(point)
        time += tparam/2*0.7
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, -90, 90, 0, 0, 0, -90, -90, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, -90, 90, 0, 0, 0, -90, -90, time)
        msg.points.append(point)

        time += tparam/2*0.7# 第6拍
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, -90, 90, 0, 0, 45, -90, -90, time)
        msg.points.append(point)
        time += tparam/2*0.7
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, -90, 90, 0, 0, 0, -90, -90, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, -90, 90, 0, 0, 0, -90, -90, time)
        msg.points.append(point)

        time += tparam*0.7# 第7拍
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, time)
        msg.points.append(point)
        time += tparam*0.7# 第8拍
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, time)
        msg.points.append(point)
        # 第6个八拍动作结束

        return msg
    def dance5(self,tparam):
        time = 1
        msg = self.get_msg_with_name()
        point_start = JointTrajectoryPoint()
        point_start.positions = [0 for i in range(26)]
        point_start.time_from_start = rospy.Duration(time)
        msg.points.append(point_start)
        # 第7个八拍动作开始
        time += tparam*0.7# 第1拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 0, -90, 90, 0, 0, 0, 0, 0, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 0, -90, 90, 0, 0, 0, 0, 0, time)
        msg.points.append(point)
        time += tparam*0.7# 第2拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 0, -90, 90, -90, 0, -90, -90, 0, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 0, -90, 90, -90, 0, -90, -90, 0, time)
        msg.points.append(point)

        time += tparam/2*0.7# 第3拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, -45, -90, 90, -90, 0, -90, -90, 0, time)
        self.down_leg(point,5)
        msg.points.append(point)
        time += tparam/2*0.7
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 0, -90, 90, -90, 0, -90, -90, 0, time)
        self.down_leg(point,10)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 0, -90, 90, -90, 0, -90, -90, 0, time)
        self.down_leg(point,10)
        msg.points.append(point)
        time += tparam/2*0.7# 第4拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, -45, -90, 90, -90, 0, -90, -90, 0, time)
        self.down_leg(point,5)
        msg.points.append(point)
        time += tparam/2*0.7
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 0, -90, 90, -90, 0, -90, -90, 0, time)
        self.down_leg(point,0)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 0, -90, 90, -90, 0, -90, -90, 0, time)
        self.down_leg(point,0)
        msg.points.append(point)

        time += tparam*0.7# 第5拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 90, -90, 0, -90, 0, 0, 0, -90, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 90, -90, 0, -90, 0, 0, 0, -90, time)
        msg.points.append(point)
        time += tparam*0.7# 第6拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 90, -90, 0, -90, 0, 0, -90, -90, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 90, -90, 0, -90, 0, 0, -90, -90, time)
        msg.points.append(point)

        time += tparam/2*0.7# 第7拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 90, -90, 0, -90, 0,45, -90, -90, time)
        self.down_leg(point,5)
        msg.points.append(point)
        time += tparam/2*0.7
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 90, -90, 0, -90, 0, 0, -90, -90, time)
        self.down_leg(point,10)
        msg.points.append(point)
        time += tparam/2*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 90, -90, 0, -90, 0, 0, -90, -90, time)
        self.down_leg(point,10)
        msg.points.append(point)
        time += tparam/2*0.7# 第8拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 90, -90, 0, -90, 0,45, -90, -90, time)
        self.down_leg(point,5)
        msg.points.append(point)
        time += tparam/2*0.7
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 90, -90, 0, -90, 0, 0, -90, -90, time)
        self.down_leg(point,0)
        msg.points.append(point)
        time += tparam/2*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 90, -90, 0, -90, 0, 0, -90, -90, time)
        self.down_leg(point,0)
        msg.points.append(point)
        # 第7个八拍动作结束
        return msg
    def dance6(self,tparam):
        time = 1
        msg = self.get_msg_with_name()
        point_start = JointTrajectoryPoint()
        point_start.positions = [0 for i in range(26)]
        point_start.time_from_start = rospy.Duration(time)
        msg.points.append(point_start)
        # 第8个八拍动作开始
        time += tparam*0.7# 第1拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 0, -90, 90, -90, 0, 0, -90, -90, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 0, -90, 90, -90, 0, 0, -90, -90, time)
        msg.points.append(point)
        time += tparam*0.7# 第2拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 0, -90, 90, 0, 0, 0, 0, -90, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 0, -90, 90, 0, 0, 0, 0, -90, time)
        msg.points.append(point)
        time += tparam*0.7# 第3拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 0, -90, 90, -90, 0, 0, -90, -90, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 0, -90, 90, -90, 0, 0, -90, -90, time)
        msg.points.append(point)

        time += tparam*0.7# 第4拍
        point = copy.deepcopy(point_start)
        point.time_from_start = rospy.Duration(time)
        self.set_the_rad(point, -30, 'waist_yaw')
        self.move_one(point, -90, 0, 0, -90, 90, -90, 0, 0, -90, -90, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        point.time_from_start = rospy.Duration(time)
        self.set_the_rad(point, -30, 'waist_yaw')
        self.move_one(point, -90, 0, 0, -90, 90, -90, 0, 0, -90, -90, time)
        msg.points.append(point)
        time += tparam*0.7# 第5拍
        point = copy.deepcopy(point_start)
        point.time_from_start = rospy.Duration(time)
        self.set_the_rad(point, 0, 'waist_yaw')
        self.move_one(point, -90, 0, 0, -90, 90, -90, 0, 0, -90, -90, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        point.time_from_start = rospy.Duration(time)
        self.set_the_rad(point, 0, 'waist_yaw')
        self.move_one(point, -90, 0, 0, -90, 90, -90, 0, 0, -90, -90, time)
        msg.points.append(point)
        time += tparam*0.7# 第6拍
        point = copy.deepcopy(point_start)
        point.time_from_start = rospy.Duration(time)
        self.set_the_rad(point, 30, 'waist_yaw')
        self.move_one(point, -90, 0, 0, -90, 90, -90, 0, 0, -90, -90, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        point.time_from_start = rospy.Duration(time)
        self.set_the_rad(point, 30, 'waist_yaw')
        self.move_one(point, -90, 0, 0, -90, 90, -90, 0, 0, -90, -90, time)
        msg.points.append(point)
        time += tparam*0.7# 第7拍
        point = copy.deepcopy(point_start)
        point.time_from_start = rospy.Duration(time)
        self.set_the_rad(point, 0, 'waist_yaw')
        self.move_one(point, -90, 0, 0, -90, 90, -90, 0, 0, -90, -90, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        point.time_from_start = rospy.Duration(time)
        self.set_the_rad(point, 0, 'waist_yaw')
        self.move_one(point, -90, 0, 0, -90, 90, -90, 0, 0, -90, -90, time)
        msg.points.append(point)

        time += tparam*0.7# 第8拍
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, time)
        msg.points.append(point)
        # 第8个八拍动作结束
        return msg
    def dance8(self,tparam):
        time = 1.0
        msg = self.get_msg_with_name()
        point_start = JointTrajectoryPoint()
        point_start.positions = [0 for i in range(26)]
        self.down_leg(point_start,10)
        point_start.time_from_start = rospy.Duration(time)
        msg.points.append(point_start)

        time += 1
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, time)
        self.down_leg(point,15)
        msg.points.append(point)

        # 第1个八拍动作开始
        time +=tparam*0.7   # 第1拍
        point= copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 0, 0, 0, 0, 0, 0, 0, 0, time)
        msg.points.append(point)
        time +=tparam*0.3
        point= copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 0, 0, 0, 0, 0, 0, 0, 0, time)
        msg.points.append(point)

        time += tparam*0.7    # 第2拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 0, -90, 0, 0, 0, 0, 0, 0, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 0, -90, 0, 0, 0, 0, 0, 0, time)
        msg.points.append(point)

        time += tparam*0.7   # 第3拍
        point= copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 0, 0, 0, 0, 0, 0, 0, 0, time)
        msg.points.append(point)
        time += tparam*0.3
        point= copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 0, 0, 0, 0, 0, 0, 0, 0, time)
        msg.points.append(point)

        time += tparam*0.7   # 第4拍
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, time)
        self.down_leg(point,15)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, time)
        self.down_leg(point,15)
        msg.points.append(point)

        time += tparam*0.7   # 第5拍
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, -90, 0, 0, 0, 0, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, -90, 0, 0, 0, 0, time)
        msg.points.append(point)

        time += tparam*0.7   # 第6拍
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, -90, 0, 0, -90, 0, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, -90, 0, 0, -90, 0, time)
        msg.points.append(point)

        time += tparam*0.7   # 第7拍
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, -90, 0, 0, 0, 0, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, -90, 0, 0, 0, 0, time)
        msg.points.append(point)

        time += tparam*0.7   # 第8拍
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, time)
        self.down_leg(point,15)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, time)
        self.down_leg(point,15)
        msg.points.append(point)
        # 第1个八拍动作结束
        # 第2个八拍动作开始
        time +=tparam*0.7   # 第1拍
        point= copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 0, 0, 0, -90, 0, 0, 0, 0, time)
        msg.points.append(point)
        time +=tparam*0.3
        point= copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 0, 0, 0, -90, 0, 0, 0, 0, time)
        msg.points.append(point)
        time += tparam*0.7   # 第2拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 0, 0, -90, -90, 0, 0, 0, 90, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 0, 0, -90, -90, 0, 0, 0, 90, time)
        msg.points.append(point)
        time += tparam*0.7   # 第3拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 90, -90, 0, -90, 0, 0, 0, 90, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 90, -90, 0, -90, 0, 0, 0, 90, time)
        msg.points.append(point)
        time += tparam*0.7   # 第4拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 90, -90, 0, -90, 0, 0, -90, 90, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 90, -90, 0, -90, 0, 0, -90, 90, time)
        msg.points.append(point)
        time += tparam/2*0.7   # 第5拍
        point = copy.deepcopy(point_start)
        self.set_the_rad(point, 30, 'neck_pitch')
        self.move_one(point, -90, 0, 90, -90, 0, -90, 0, 0, -90, 90, time)
        msg.points.append(point)
        time += tparam/2*0.7
        point = copy.deepcopy(point_start)
        self.set_the_rad(point, 0, 'neck_pitch')
        self.move_one(point, -90, 0, 90, -90, 0, -90, 0, 0, -90, 90, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.set_the_rad(point, 0, 'neck_pitch')
        self.move_one(point, -90, 0, 90, -90, 0, -90, 0, 0, -90, 90, time)
        msg.points.append(point)
        time += tparam/2*0.7   # 第6拍
        point = copy.deepcopy(point_start)
        self.set_the_rad(point, 30, 'neck_pitch')
        self.move_one(point, -90, 0, 90, -90, 0, -90, 0, 0, -90, 90, time)
        msg.points.append(point)
        time += tparam/2*0.7
        point = copy.deepcopy(point_start)
        self.set_the_rad(point, 0, 'neck_pitch')
        self.move_one(point, -90, 0, 90, -90, 0, -90, 0, 0, -90, 90, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.set_the_rad(point, 0, 'neck_pitch')
        self.move_one(point, -90, 0, 90, -90, 0, -90, 0, 0, -90, 90, time)
        msg.points.append(point)
        time += tparam*0.7   # 第7拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -30, 0, 90, -90, 0, -45, 0, 0, -90, 90, time)
        self.down_leg(point,15)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -30, 0, 90, -90, 0, -45, 0, 0, -90, 90, time)
        self.down_leg(point,15)
        msg.points.append(point)
        time += tparam*1   # 第8拍
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, time)
        self.down_leg(point,15)
        msg.points.append(point)
        # 第2个八拍动作结束
        # 第3个八拍动作开始
        time += tparam*0.7   # 第1拍
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 90, 90, 0, -90, 0, 0, 0, 0, 0, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 90, 90, 0, -90, 0, 0, 0, 0, 0, time)
        msg.points.append(point)
        time += tparam*0.7   # 第2拍
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 90, 90, -90, 0, 0, 0, 0, 0, 0, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 90, 90, -90, 0, 0, 0, 0, 0, 0, time)
        msg.points.append(point)
        time += tparam*0.7   # 第3拍
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 90, 90, -90, 0, 0, -90, -90, 0, 90, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 90, 90, -90, 0, 0, -90, -90, 0, 90, time)
        msg.points.append(point)

        time += tparam*0.7   # 第4拍
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 90, 90, -90, 0, 0, -90, -90, -90, 0, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 90, 90, -90, 0, 0, -90, -90, -90, 0, time)
        msg.points.append(point)
        time += tparam/2*0.7   # 第5拍
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 90, 0, -90, 0, 0, -90, -90, -90, 0, time)
        msg.points.append(point)
        time += tparam/2*0.7
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 90, 90, -90, 0, 0, -90, -90, -90, 0, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 90, 90, -90, 0, 0, -90, -90, -90, 0,time)
        msg.points.append(point)
        time += tparam/2*0.7   # 第6拍
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 90, 0, -90, 0, 0, -90, -90, -90, 0,time)
        msg.points.append(point)
        time += tparam/2*0.7
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 90, 90, -90, 0,0, -90, -90, -90, 0, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 90, 90, -90, 0, 0, -90, -90, -90, 0,time)
        msg.points.append(point)
        time += tparam*0.7   # 第7拍
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 90, 90, 0, -90, 0, -90, -90, 0, 90, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 90, 90, 0, -90, 0, -90, -90, 0, 90, time)
        msg.points.append(point)
        time += tparam*0.7   # 第8拍
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, time)
        msg.points.append(point)

        # 第3个八拍动作结束
        # 第4个八拍动作开始
        time +=tparam*0.7   # 第1拍
        point= copy.deepcopy(point_start)
        self.move_one(point, -45, 0, 0, -90, 0,  -45, -30, 0, -90, 0, time)
        msg.points.append(point)
        time +=tparam*0.3
        point= copy.deepcopy(point_start)
        self.move_one(point, -45, 0, 0, -90, 0,  -45, -30, 0, -90, 0, time)
        msg.points.append(point)
        time += tparam*0.7   # 第2拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -60, 15, 0, -75, 0,  -60, -15, 0, -75, 0, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -60, 15, 0, -75, 0,  -60, -15, 0, -75, 0, time)
        msg.points.append(point)
        time += tparam*0.7   # 第3拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -45, 30, 0, -60, 0,  -45, 0, 0, -60, 0, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -45, 30, 0, -60, 0,  -45, 0, 0, -60, 0, time)
        msg.points.append(point)
        time += tparam*0.7   # 第4拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -30, 15, 0, -75, 0,  -30, -15, 0, -75, 0, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -30, 15, 0, -75, 0,  -30, -15, 0, -75, 0, time)
        msg.points.append(point)

        time += tparam*0.7   # 第5拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -45, 0, 0, -90, 0, -45, -30, 0, -90, 0, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -45, 0, 0, -90, 0, -45, -30, 0, -90, 0, time)
        msg.points.append(point)
        time += tparam*0.7   # 第6拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -60, 15, 0, -75, 0, -60, -15, 0, -75, 0, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -60, 15, 0, -75, 0, -60, -15, 0, -75, 0, time)
        msg.points.append(point)
        time += tparam*0.7   # 第7拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -45, 30, 0, -60, 0, -45, 0, 0, -60, 0, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -45, 30, 0, -60, 0, -45, 0, 0, -60, 0, time)
        msg.points.append(point)
        time += tparam*0.7   # 第8拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -30, 15, 0, -75, 0, -30, -15, 0, -75, 0, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -30, 15, 0, -75, 0, -30, -15, 0, -75, 0, time)
        msg.points.append(point)
        # 第4个八拍动作结束
        # 第5个八拍动作开始
        time += tparam*0.7# 第1拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -50, 10, 0, -90, 90, 0, 0, 0, 0, 0, time)
        self.down_leg(point,15)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -50, 10, 0, -90, 90, 0, 0, 0, 0, 0, time)
        self.down_leg(point,15)
        msg.points.append(point)
        time += tparam*0.7# 第2拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -50, 10, 0, -90, 90, -50, -10, 0, -90, -90, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -50, 10, 0, -90, 90, -50, -10, 0, -90, -90, time)
        msg.points.append(point)

        time += tparam/2*0.7# 第3拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -30, 10, 0, -90, 90, -50, -10, 0, -90, -90, time)
        msg.points.append(point)
        time += tparam/2*0.7
        point = copy.deepcopy(point_start)
        self.move_one(point, -50, 10, 0, -90, 90, -50, -10, 0, -90, -90, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -50, 10, 0, -90, 90, -50, -10, 0, -90, -90, time)
        msg.points.append(point)
        time += tparam/2*0.7# 第4拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -30, 10, 0, -90, 90, -50, -10, 0, -90, -90, time)
        msg.points.append(point)
        time += tparam/2*0.7
        point = copy.deepcopy(point_start)
        self.move_one(point, -50, 10, 0, -90, 90, -50, -10, 0, -90, -90, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -50, 10, 0, -90, 90, -50, -10, 0, -90, -90, time)
        msg.points.append(point)

        time += tparam/2*0.7# 第5拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -50, 10, 0, -90, 90, -30, -10, 0, -90, -90, time)
        msg.points.append(point)
        time += tparam/2*0.7
        point = copy.deepcopy(point_start)
        self.move_one(point, -50, 10, 0, -90, 90, -50, -10, 0, -90, -90, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -50, 10, 0, -90, 90, -50, -10, 0, -90, -90, time)
        msg.points.append(point)
        time += tparam/2*0.7# 第6拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -50, 10, 0, -90, 90, -30, -10, 0, -90, -90, time)
        msg.points.append(point)
        time += tparam/2*0.7
        point = copy.deepcopy(point_start)
        self.move_one(point, -50, 10, 0, -90, 90, -50, -10, 0, -90, -90, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -50, 10, 0, -90, 90, -50, -10, 0, -90, -90, time)
        msg.points.append(point)

        time += tparam*0.7# 第7拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -25, 10, 0, -90, 90, -25, -10, 0, -90, -90, time)
        self.down_leg(point,12)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -25, 10, 0, -90, 90, -25, -10, 0, -90, -90, time)
        self.down_leg(point,12)
        msg.points.append(point)
        time += tparam*0.7# 第8拍
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, time)
        self.down_leg(point,15)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, time)
        self.down_leg(point,15)
        msg.points.append(point)
        # 第5个八拍动作结束
        # 第6个八拍动作开始
        time += tparam*0.7# 第1拍
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, -90, -90, -90, 0, 0, 0, 0, 0, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, -90, -90, -90, 0, 0, 0, 0, 0, time)
        msg.points.append(point)

        time += tparam*0.7# 第2拍
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, -90, 90, 0, 0, 0, 0, 0, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, -90, 90, 0, 0, 0, 0, 0, time)
        msg.points.append(point)

        time += tparam*0.7# 第3拍
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, -90, 90, 0, 0, 90, -90, 90, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, -90, 90, 0, 0, 90, -90, 90, time)
        msg.points.append(point)

        time += tparam*0.7# 第4拍
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, -90, 90, 0, 0, 0, -90, -90, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, -90, 90, 0, 0, 0, -90, -90, time)
        msg.points.append(point)

        time += tparam/2*0.7# 第5拍
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, -45, -90, 90, 0, 0, 0, -90, -90, time)
        msg.points.append(point)
        time += tparam/2*0.7
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, -90, 90, 0, 0, 0, -90, -90, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, -90, 90, 0, 0, 0, -90, -90, time)
        msg.points.append(point)

        time += tparam/2*0.7# 第6拍
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, -90, 90, 0, 0, 45, -90, -90, time)
        msg.points.append(point)
        time += tparam/2*0.7
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, -90, 90, 0, 0, 0, -90, -90, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, -90, 90, 0, 0, 0, -90, -90, time)
        msg.points.append(point)

        time += tparam*0.7# 第7拍
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, time)
        msg.points.append(point)
 
        '''
        time += tparam*0.7# 第8拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -45, 0, 0, -90, 90, 0, 0, 0, 0, 0, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -45, 0, 0, -90, 90, 0, 0, 0, 0, 0, time)
        msg.points.append(point)
        '''
        # 第6个八拍动作结束
        # 第7个八拍动作开始
        time += tparam*1.4# 第1拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 0, -90, 90, 0, 0, 0, 0, 0, time)
        self.down_leg(point,5)
        msg.points.append(point)
        time += tparam*0.6
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 0, -90, 90, 0, 0, 0, 0, 0, time)
        self.down_leg(point,5)
        msg.points.append(point)
        time += tparam*0.7# 第2拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 0, -90, 90, -90, 0, -90, -90, 0, time)
        self.down_leg(point,5)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 0, -90, 90, -90, 0, -90, -90, 0, time)
        self.down_leg(point,5)
        msg.points.append(point)

        time += tparam/2*0.7# 第3拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, -45, -90, 90, -90, 0, -90, -90, 0, time)
        # self.down_leg(point,15)
        msg.points.append(point)
        time += tparam/2*0.7
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 0, -90, 90, -90, 0, -90, -90, 0, time)
        # self.down_leg(point,15)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 0, -90, 90, -90, 0, -90, -90, 0, time)
        # self.down_leg(point,15)
        msg.points.append(point)
        time += tparam/2*0.7# 第4拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, -45, -90, 90, -90, 0, -90, -90, 0, time)
        msg.points.append(point)
        time += tparam/2*0.7
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 0, -90, 90, -90, 0, -90, -90, 0, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 0, -90, 90, -90, 0, -90, -90, 0, time)
        msg.points.append(point)

        time += tparam*0.7# 第5拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 90, -90, 0, -90, 0, 0, 0, -90, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 90, -90, 0, -90, 0, 0, 0, -90, time)
        msg.points.append(point)
        time += tparam*0.7# 第6拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 90, -90, 0, -90, 0, 0, -90, -90, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 90, -90, 0, -90, 0, 0, -90, -90, time)
        msg.points.append(point)

        time += tparam/2*0.7# 第7拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 90, -90, 0, -90, 0,45, -90, -90, time)
        # self.down_leg(point,12)
        msg.points.append(point)
        time += tparam/2*0.7
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 90, -90, 0, -90, 0, 0, -90, -90, time)
        # self.down_leg(point,15)
        msg.points.append(point)
        time += tparam/2*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 90, -90, 0, -90, 0, 0, -90, -90, time)
        # self.down_leg(point,15)
        msg.points.append(point)
        time += tparam/2*0.7# 第8拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 90, -90, 0, -90, 0,45, -90, -90, time)
        # self.down_leg(point,12)
        msg.points.append(point)
        time += tparam/2*0.7
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 90, -90, 0, -90, 0, 0, -90, -90, time)
        msg.points.append(point)
        time += tparam/2*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 90, -90, 0, -90, 0, 0, -90, -90, time)
        msg.points.append(point)
        # 第7个八拍动作结束
        # 第8个八拍动作开始
        time += tparam*0.7# 第1拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 0, -90, 90, -90, 0, 0, -90, -90, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 0, -90, 90, -90, 0, 0, -90, -90, time)
        msg.points.append(point)
        time += tparam*0.7# 第2拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 0, -90, 90, 0, 0, 0, 0, -90, time)
        # self.down_leg(point,15)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 0, -90, 90, 0, 0, 0, 0, -90, time)
        # self.down_leg(point,15)
        msg.points.append(point)
        time += tparam*0.7# 第3拍
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 0, -90, 90, -90, 0, 0, -90, -90, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 0, -90, 90, -90, 0, 0, -90, -90, time)
        msg.points.append(point)

        time += tparam*0.7# 第4拍
        point = copy.deepcopy(point_start)
        point.time_from_start = rospy.Duration(time)
        self.set_the_rad(point, -30, 'waist_yaw')
        self.move_one(point, -90, 0, 0, -90, 90, -90, 0, 0, -90, -90, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        point.time_from_start = rospy.Duration(time)
        self.set_the_rad(point, -30, 'waist_yaw')
        self.move_one(point, -90, 0, 0, -90, 90, -90, 0, 0, -90, -90, time)
        msg.points.append(point)
        time += tparam*0.7# 第5拍
        point = copy.deepcopy(point_start)
        point.time_from_start = rospy.Duration(time)
        self.set_the_rad(point, 0, 'waist_yaw')
        self.move_one(point, -90, 0, 0, -90, 90, -90, 0, 0, -90, -90, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        point.time_from_start = rospy.Duration(time)
        self.set_the_rad(point, 0, 'waist_yaw')
        self.move_one(point, -90, 0, 0, -90, 90, -90, 0, 0, -90, -90, time)
        msg.points.append(point)
        time += tparam*0.7# 第6拍
        point = copy.deepcopy(point_start)
        point.time_from_start = rospy.Duration(time)
        self.set_the_rad(point, 30, 'waist_yaw')
        self.move_one(point, -90, 0, 0, -90, 90, -90, 0, 0, -90, -90, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        point.time_from_start = rospy.Duration(time)
        self.set_the_rad(point, 30, 'waist_yaw')
        self.move_one(point, -90, 0, 0, -90, 90, -90, 0, 0, -90, -90, time)
        msg.points.append(point)
        time += tparam*0.7# 第7拍
        point = copy.deepcopy(point_start)
        point.time_from_start = rospy.Duration(time)
        self.set_the_rad(point, 0, 'waist_yaw')
        self.move_one(point, -90, 0, 0, -90, 90, -90, 0, 0, -90, -90, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        point.time_from_start = rospy.Duration(time)
        self.set_the_rad(point, 0, 'waist_yaw')
        self.move_one(point, -90, 0, 0, -90, 90, -90, 0, 0, -90, -90, time)
        msg.points.append(point)
        time += tparam*0.7# 第8拍
        point = copy.deepcopy(point_start)
        point.time_from_start = rospy.Duration(time)
        self.set_the_rad(point, 0, 'waist_yaw')
        self.move_one(point, -90, 0, 0, 0, 90, -90, 0, 0, 0, -90, time)
        msg.points.append(point)
        time += tparam*0.3
        point = copy.deepcopy(point_start)
        point.time_from_start = rospy.Duration(time)
        self.set_the_rad(point, 0, 'waist_yaw')
        self.move_one(point, -90, 0, 0, 0, 90, -90, 0, 0, 0, -90, time)
        msg.points.append(point)
        # 第8个八拍动作结束
        # 第9个八拍动作开始
        time +=tparam  # 第1拍
        point= copy.deepcopy(point_start)
        self.move_one(point, -90, 0, -30, -90, 90, -90, 0, -30, -90, -90, time)
        self.tilt_leg(point,5)
        msg.points.append(point)
        time +=tparam  # 第2拍
        point= copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 30, -90, 90, -90, 0, 30, -90, -90, time)
        self.tilt_leg(point,-5)
        msg.points.append(point)
        time +=tparam # 第3拍
        point= copy.deepcopy(point_start)
        self.move_one(point, -90, 0, -30, -90, 90, -90, 0, -30, -90, -90, time)
        self.tilt_leg(point,5)
        msg.points.append(point)
        time +=tparam # 第4拍
        point= copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 30, -90, 90, -90, 0, 30, -90, -90, time)
        self.tilt_leg(point,-5)
        msg.points.append(point)
        time +=tparam # 第5拍
        point= copy.deepcopy(point_start)
        self.move_one(point, -90, 0, -30, -90, 90, -90, 0, -30, -90, -90, time)
        self.tilt_leg(point,5)
        msg.points.append(point)
        time +=tparam # 第6拍
        point= copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 30, -90, 90, -90, 0, 30, -90, -90, time)
        self.tilt_leg(point,-5)
        msg.points.append(point)
        time +=tparam # 第7拍
        point= copy.deepcopy(point_start)
        self.move_one(point, -90, 0, -30, -90, 90, -90, 0, -30, -90, -90, time)
        self.tilt_leg(point,5)
        msg.points.append(point)
        time +=tparam # 第8拍
        point= copy.deepcopy(point_start)
        self.move_one(point, -90, 0, 30, -90, 90, -90, 0, 30, -90, -90, time)
        self.tilt_leg(point,-5)
        msg.points.append(point)
        # 第9个八拍动作结束
        time +=3 
        point= copy.deepcopy(point_start)
        self.move_one(point, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, time)
        self.tilt_leg(point,0)
        self.down_leg(point,0)
        msg.points.append(point)
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

    def wasit_twist(self, twist_rad):
        msg = self.get_msg_with_name()
        point_start = JointTrajectoryPoint()
        point_start.positions = [0 for i in range(26)]
        point_start.time_from_start = rospy.Duration(1)
        msg.points.append(point_start)

        point1 = JointTrajectoryPoint()
        point1.positions = [0 for i in range(26)]
        self.set_the_rad(point1, -80, 'r_shoulder_pitch')
        self.set_the_rad(point1, -80, 'r_elbow_pitch')
        self.set_the_rad(point1, -15, 'r_shoulder_roll')
        self.set_the_rad(point1, -80, 'l_shoulder_pitch')
        self.set_the_rad(point1, -80, 'l_elbow_pitch')
        self.set_the_rad(point1, 15, 'l_shoulder_roll')
        point1.time_from_start = rospy.Duration(5)
        msg.points.append(point1)

        point2 = copy.deepcopy(point1)
        point2.time_from_start = rospy.Duration(6)
        self.set_the_rad(point2, twist_rad, 'waist_yaw')
        msg.points.append(point2)

        point3 = copy.deepcopy(point1)
        point3.time_from_start = rospy.Duration(8)
        self.set_the_rad(point3, 0, 'waist_yaw')
        msg.points.append(point3)

        point4 = copy.deepcopy(point_start)
        point4.time_from_start = rospy.Duration(13)
        msg.points.append(point4)
        return msg


    def jump(self, knee_rad, dur):
        msg = self.get_msg_with_name()
        point_start = JointTrajectoryPoint()
        point_start.positions = [0 for i in range(26)]
        point_start.time_from_start = rospy.Duration(1)
        msg.points.append(point_start)

        point1 = JointTrajectoryPoint()
        point1.positions = [0 for i in range(26)]
        self.set_the_rad(point1, knee_rad, 'r_knee_pitch')
        self.set_the_rad(point1, -1*knee_rad/2, 'r_ankle_pitch')
        self.set_the_rad(point1, -1*knee_rad/2, 'r_hip_pitch')
        self.set_the_rad(point1, knee_rad, 'l_knee_pitch')
        self.set_the_rad(point1, -1*knee_rad/2, 'l_ankle_pitch')
        self.set_the_rad(point1, -1*knee_rad/2, 'l_hip_pitch')
        point1.time_from_start = rospy.Duration(5)
        msg.points.append(point1)

        point4 = copy.deepcopy(point_start)
        point4.time_from_start = rospy.Duration(5+dur)
        msg.points.append(point4)
        return msg


    def stand_up(self, dur):
        point = JointTrajectoryPoint()
        point.positions = [0 for i in range(26)]
        point.time_from_start = rospy.Duration(dur)
        return point

    def squat(self, knee_rad, dur):
        point = JointTrajectoryPoint()
        point.positions = [0 for i in range(26)]
        self.set_the_rad(point, knee_rad, 'r_knee_pitch')
        self.set_the_rad(point, -1*knee_rad/2, 'r_ankle_pitch')
        self.set_the_rad(point, -1*knee_rad/2, 'r_hip_pitch')
        self.set_the_rad(point, knee_rad, 'l_knee_pitch')
        self.set_the_rad(point, -1*knee_rad/2, 'l_ankle_pitch')
        self.set_the_rad(point, -1*knee_rad/2, 'l_hip_pitch')
        point.time_from_start = rospy.Duration(5)
        return point

    def lean(self, s_point, deg, dur):
        point = copy.deepcopy(s_point)
        self.set_the_rad(point, deg, 'r_hip_roll')
        self.set_the_rad(point, deg, 'l_hip_roll')
        self.set_the_rad(point, -deg, 'r_ankle_roll')
        self.set_the_rad(point, -deg, 'l_ankle_roll')
        point.time_from_start = rospy.Duration(dur)
        return point

    def raise_foot(self, s_point, knee_deg, lr, dur):
        point = copy.deepcopy(s_point)
        self.set_the_rad(point, knee_deg, lr + '_knee_pitch')
        self.set_the_rad(point, -1*knee_deg/2, lr + '_ankle_pitch')
        self.set_the_rad(point, -1*knee_deg/2, lr + '_hip_pitch')
        point.time_from_start = rospy.Duration(dur)
        return point

    def turn_hip(self, s_point, deg, lr, dur):
        point = copy.deepcopy(s_point)
        self.set_the_rad(point, deg, lr + '_hip_yaw')
        point.time_from_start = rospy.Duration(dur)
        return point

    def raise_rfoot_turn_lhip(self, s_point, knee_deg, turn_deg, dur):
        point = copy.deepcopy(s_point)
        self.set_the_rad(point, knee_deg, 'r_knee_pitch')
        self.set_the_rad(point, -1*knee_deg/2, 'r_ankle_pitch')
        self.set_the_rad(point, -1*knee_deg/2, 'r_hip_pitch')
        self.set_the_rad(point, turn_deg, 'l_hip_yaw')
        point.time_from_start = rospy.Duration(dur)
        return point


    def lower_foot(self, s_point, knee_deg, lr, dur):
        point = copy.deepcopy(s_point)
        self.set_the_rad(point, knee_deg, lr + '_knee_pitch')
        self.set_the_rad(point, -1*knee_deg/2, lr + '_ankle_pitch')
        self.set_the_rad(point, -1*knee_deg/2, lr + '_hip_pitch')
        point.time_from_start = rospy.Duration(dur)
        return point

    def stand(self, s_point, dur):
        point = copy.deepcopy(s_point)
        self.set_the_rad(point, 0, 'r_hip_roll')
        self.set_the_rad(point, 0, 'l_hip_roll')
        self.set_the_rad(point, 0, 'r_ankle_roll')
        self.set_the_rad(point, 0, 'l_ankle_roll')
        point.time_from_start = rospy.Duration(dur)
        return point

    def turn_rhip_1(self, s_point, knee_deg, dur):
        rhip_deg = -40
        rknee_deg = -rhip_deg
        point = copy.deepcopy(s_point)
        self.set_the_rad(point, 90, 'r_hip_yaw')
        self.set_the_rad(point, 0, 'r_hip_roll')
        self.set_the_rad(point, 0, 'r_ankle_roll')

        self.set_the_rad(point, 40, 'l_knee_pitch')
        self.set_the_rad(point, -20, 'l_ankle_pitch')
        self.set_the_rad(point, -20, 'l_hip_pitch')

        self.set_the_rad(point, rknee_deg, 'r_knee_pitch')
        self.set_the_rad(point, 0, 'r_ankle_pitch')
        self.set_the_rad(point, rhip_deg, 'r_hip_pitch')

        point.time_from_start = rospy.Duration(dur)
        return point

    def turn_rhip_2(self, s_point, knee_deg, dur):
        point = copy.deepcopy(s_point)
        self.set_the_rad(point, 10, 'l_hip_roll')
        self.set_the_rad(point, -10, 'l_ankle_roll')

        self.set_the_rad(point, 40, 'l_knee_pitch')
        self.set_the_rad(point, -25, 'l_ankle_pitch')
        self.set_the_rad(point, -15, 'l_hip_pitch')

        self.set_the_rad(point, 40, 'r_knee_pitch')
        self.set_the_rad(point, -30, 'r_ankle_pitch')
        self.set_the_rad(point, -10, 'r_hip_pitch')

        point.time_from_start = rospy.Duration(dur)
        return point

    def turn_rhip_3(self, s_point, knee_deg, dur):
        point = copy.deepcopy(s_point)
        self.set_the_rad(point, 0, 'r_knee_pitch')
        self.set_the_rad(point, 0, 'r_ankle_pitch')
        self.set_the_rad(point, 0, 'r_hip_pitch')

        self.set_the_rad(point, 15, 'r_hip_roll')
        self.set_the_rad(point, -15, 'r_ankle_roll')
        self.set_the_rad(point, 15, 'l_hip_roll')
        self.set_the_rad(point, -15, 'l_ankle_roll')

        self.set_the_rad(point, 0, 'r_hip_yaw')
        point.time_from_start = rospy.Duration(dur)
        return point




    def turn_left(self, deg):
        msg = self.get_msg_with_name()

        stand_pos = self.stand_up(1)
        msg.points.append(stand_pos)

        #squat_pos = self.squat(40, 5)
        #msg.points.append(squat_pos)

        #lean_left_pos = self.lean(squat_pos, -15, 6)
        pos_1 = self.lean(stand_pos, -15, 3)
        msg.points.append(pos_1)

        pos_2 = self.raise_foot(pos_1, 40, 'r', 5)
        msg.points.append(pos_2)

        pos_3 = self.turn_rhip_1(pos_2, 10, 6)
        msg.points.append(pos_3)

        pos_4 = self.turn_rhip_2(pos_3, 10, 6.7)
        msg.points.append(pos_4)

        pos_6 = self.turn_rhip_3(pos_4, 10, 7)
        msg.points.append(pos_6)


        '''
        #turn_lhip_pos = self.raise_rfoot_turn_lhip(lean_left_pos, 70, 10, 14)
        #msg.points.append(turn_lhip_pos)

        pos_4 = self.lower_foot(pos_3, 0, 'r', 7)
        msg.points.append(pos_4)

        pos_5 = self.stand(pos_4, 8)
        msg.points.append(pos_5)

        pos_6 = self.lean(pos_5, 15, 9)
        msg.points.append(pos_6)

        pos_7 = self.raise_foot(pos_6, 30, 'l', 10)
        msg.points.append(pos_7)

        pos_8 = self.turn_hip(pos_7, 0, 'l', 11)
        msg.points.append(pos_8)

        pos_9 = self.lower_foot(pos_8, 0, 'l', 12)
        msg.points.append(pos_9)
        '''

        point_end = self.stand_up(7.5)
        msg.points.append(point_end)

        return msg



    def get_move_msg(self,angles):

        msg = self.get_msg_with_name()
        point_start = JointTrajectoryPoint()
        point_start.positions = [0 for i in range(26)]
        point_start.time_from_start = rospy.Duration(0.1)
        msg.points.append(point_start)

        point = JointTrajectoryPoint()
        print(angles)
        point.positions = angles
        point.time_from_start = rospy.Duration(1)
        msg.points.append(point)
        return msg


if __name__ == '__main__':

    do = ""

    try:
        opts, args = getopt.getopt(sys.argv[1:],"d:",["do"])
    except getopt.GetoptError:
        print '-d behaviors'
        sys.exit(2)

    for opt, arg in opts:
        if opt in ('-d', "--do"):
            do = arg

    pub = rospy.Publisher('/arobot/arobot_trajectory_controller/command', JointTrajectory,queue_size=1000)

    rospy.init_node('control_demo_trajectory_pub', anonymous = False)

    demo = robotDemo()

    rospy.sleep(3)

    r = rospy.Rate(5);


    if do in ("wave", "wave_hand"):
        pub.publish(demo.wave_hand())
    elif do in ("jump"):
        pub.publish(demo.jump(60,5))
    elif do in ("wasit_twist", "twist"):
        pub.publish(demo.wasit_twist(-15))
        rospy.sleep(15)
        pub.publish(demo.wasit_twist(+15))
    elif do in ("hand"):
        pub.publish(demo.hand(1.0))
    elif do in ("ceshi"):
        pub.publish(demo.ceshi(5.0))
    elif do in ("dance0"):
        pub.publish(demo.dance0(1.0))
    elif do in ("dance1"):
        pub.publish(demo.dance1(1.0))
    elif do in ("dance2"):
        pub.publish(demo.dance2(1.0))
    elif do in ("dance3"):
        pub.publish(demo.dance3(1.0))
    elif do in ("dance4"):
        pub.publish(demo.dance4(1.0))
    elif do in ("dance5"):
        pub.publish(demo.dance5(1.0))
    elif do in ("dance6"):
        pub.publish(demo.dance6(1.0))
    elif do in ("dance7"):
        pub.publish(demo.dance7(1.0))
    elif do in ("dance_circle"):
        pub.publish(demo.dance_circle(1.0))
    elif do in ("dance12"):# dance10表示8组舞蹈动作单拍执行时间是1.0秒
        pub.publish(demo.dance8(1.2))
    elif do in ("dance15"):# dance15表示8组舞蹈动作单拍执行时间是1.5秒
        pub.publish(demo.dance8(1.5))
    elif do in ("dance20"):# dance20表示8组舞蹈动作单拍执行时间是2.0秒
        pub.publish(demo.dance8(2.0))
    elif do in ("keep_dance"):
        while not rospy.is_shutdown():
            pub.publish(demo.dance8(1.2))
            rospy.sleep(100)
    elif do in ("throw"):
        pub.publish(demo.throw())
    elif do in ("move_arm", "arm"):
        pub.publish(demo.move_two_arm())
        rospy.sleep(4.5)
        pub.publish(demo.move_two_arm())
        rospy.sleep(4.5)
        pub.publish(demo.move_two_arm())
    elif do in ("keep_move_arm"):
        while not rospy.is_shutdown():
            pub.publish(demo.move_two_arm())
            r.sleep()
