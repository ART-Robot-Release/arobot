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
        point_start.positions = [0 for i in range(12)]
        point_start.time_from_start = rospy.Duration(4)
        msg.points.append(point_start)

        return msg

    def move_left(self,point,angle1,angle2,angle3,angle4,angle5):

        self.set_the_rad(point,angle1, 'l_hip_roll')
        self.set_the_rad(point,angle2, 'l_hip_pitch')
        self.set_the_rad(point,angle3, 'l_knee_pitch')
        self.set_the_rad(point,angle4, 'l_ankle_pitch')
        self.set_the_rad(point, angle5, 'l_ankle_roll')
    def move_right(self,point,angle1,angle2,angle3,angle4,angle5):

        self.set_the_rad(point,angle1, 'r_hip_roll')
        self.set_the_rad(point,angle2, 'r_hip_pitch')
        self.set_the_rad(point,angle3, 'r_knee_pitch')
        self.set_the_rad(point,angle4, 'r_ankle_pitch')
        self.set_the_rad(point, angle5, 'r_ankle_roll')

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

        return msg
   
    def raise_left(self,tparam):
        time = 1
        msg = self.get_msg_with_name()
        point_start = JointTrajectoryPoint()
        point_start.positions = [0 for i in range(12)]
        point_start.time_from_start = rospy.Duration(time)
        msg.points.append(point_start)

        time +=tparam  
        point= copy.deepcopy(point_start)
        self.move_left(point,15,0,0,0,-15)
        self.move_right(point,15,0,0,0,-15)
        point.time_from_start = rospy.Duration(time)
        msg.points.append(point)

        time +=tparam  
        point= copy.deepcopy(point_start)
        self.move_left(point,15,-35,70,-35,-15)
        self.move_right(point,15,0,0,0,-15)
        point.time_from_start = rospy.Duration(time)
        msg.points.append(point)
      
        return msg

    def raise_right(self,tparam):
        time = 1
        msg = self.get_msg_with_name()
        point_start = JointTrajectoryPoint()
        point_start.positions = [0 for i in range(12)]
        point_start.time_from_start = rospy.Duration(time)
        msg.points.append(point_start)

        time +=tparam
        point= copy.deepcopy(point_start)
        self.move_right(point,-15,0,0,0,15)
        self.move_left(point,-15,0,0,0,15)
        point.time_from_start = rospy.Duration(time)
        msg.points.append(point)

        time +=tparam
        point= copy.deepcopy(point_start)
        self.move_right(point,-15,-35,70,-35,15)
        self.move_left(point,-15,0,0,0,15)
        point.time_from_start = rospy.Duration(time)
        msg.points.append(point)

        return msg

    def get_move_msg(self,angles):

        msg = self.get_msg_with_name()
        point_start = JointTrajectoryPoint()
        point_start.positions = [0 for i in range(12)]
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


    if do in ("left", "raise_left"):
        pub.publish(demo.raise_left(2.0))
    elif do in ("right", "raise_right"):
        pub.publish(demo.raise_right(2.0))
    elif do in ("reset"):
        pub.publish(demo.move_to_start())
    elif do in ("keep", "keep_step"):
        while not rospy.is_shutdown():
            pub.publish(demo.raise_left(2.0))
            rospy.sleep(7)
            pub.publish(demo.move_to_start())
            rospy.sleep(4)
            pub.publish(demo.raise_right(2.0))
            rospy.sleep(7)
            pub.publish(demo.move_to_start())
            rospy.sleep(4)

