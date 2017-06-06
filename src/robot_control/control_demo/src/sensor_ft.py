#!/usr/bin/env python
# -* - coding: UTF-8 -* -
# this demo need trajectory_controller_main.launch.
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from arobot_msgs.msg import ImuBasic, ForceSix
import copy
import sys, getopt

def angleToRad(angle):
    return angle*3.1416/180

class ft :
    fx = 0
    fy = 0
    fz = 0
    tx = 0
    ty = 0
    tz = 0

class imu:
    pitch = 0
    yaw = 0
    roll = 0

class robotDemo:
    joint_names_legs = []
    joint_names_larm = []
    joint_names_rarm = []
    joint_names_waist = []
    joint_names_head = []
    joint_ids = {}
    count = 0
    a = 0.0

    imu_ = imu()
    l_ft_ = ft()
    r_ft_ = ft()
    js = JointState()

    def __init__(self):
        self.joint_names_legs = [
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
            'r_ankle_pitch'
        ]

        self.joint_names_larm = [
            'l_shoulder_pitch',
            'l_shoulder_roll',
            'l_elbow_yaw',
            'l_elbow_pitch',
            'l_wrist_yaw'
        ]

        self.joint_names_rarm = [
            'r_shoulder_pitch',
            'r_shoulder_roll',
            'r_elbow_yaw',
            'r_elbow_pitch',
            'r_wrist_yaw'
        ]

        self.joint_names_waist = [
            'waist_pitch',
            'waist_yaw'
        ]

        self.joint_names_head = [
            'neck_pitch',
            'neck_yaw'
        ]

        for i, name in enumerate(self.joint_names_legs):
            self.joint_ids[name] = i

        for i, name in enumerate(self.joint_names_larm):
            self.joint_ids[name] = i

        for i, name in enumerate(self.joint_names_rarm):
            self.joint_ids[name] = i

        for i, name in enumerate(self.joint_names_waist):
            self.joint_ids[name] = i

        for i, name in enumerate(self.joint_names_head):
            self.joint_ids[name] = i


    def set_the_rad(self, point, angle, name):
        point.positions[self.joint_ids[name]] = angleToRad(angle)

    def get_msg_with_name(self, name):
        # type: (object) -> object
        msg = JointTrajectory()
        if 'left_arm' in name:
            msg.joint_names = self.joint_names_larm
        elif 'right_arm' in name:
            msg.joint_names = self.joint_names_rarm
        elif 'head' in name:
            msg.joint_names = self.joint_names_head
        elif 'waist' in name:
            msg.joint_names = self.joint_names_waist
        else:
            msg.joint_names = self.joint_names_legs

        return msg

    def move_arms_with_ft(self):

        msgl = self.get_msg_with_name('left_arm')
        msgr = self.get_msg_with_name('right_arm')
        trigger_pub = False
        point_start = JointTrajectoryPoint()
        point_start.positions = [0 for i in range(5)]

        point1 = copy.deepcopy(point_start)
        point1.positions = [0 for i in range(5)]
        point2 = copy.deepcopy(point_start)
        point2.positions = [0 for i in range(5)]
        if self.l_ft_.fz <= -400 and self.r_ft_.fz > -100:
            self.set_the_rad(point1, 20, 'r_shoulder_pitch')
            self.set_the_rad(point1, 0, 'r_elbow_pitch')
            point1.time_from_start = rospy.Duration(1.0)
            self.set_the_rad(point2, -20, 'l_shoulder_pitch')
            self.set_the_rad(point2, -10, 'l_elbow_pitch')
            point2.time_from_start = rospy.Duration(1.0)
            trigger_pub = True
        elif self.r_ft_.fz <= -400 and self.l_ft_.fz > -100:
            self.set_the_rad(point1, -20, 'r_shoulder_pitch')
            self.set_the_rad(point1, -10, 'r_elbow_pitch')
            point1.time_from_start = rospy.Duration(1.0)
            self.set_the_rad(point2, 20, 'l_shoulder_pitch')
            self.set_the_rad(point2, 0, 'l_elbow_pitch')
            point2.time_from_start = rospy.Duration(1.0)
            trigger_pub = True

        msgl.points.append(point2)
        msgr.points.append(point1)

        return msgl, msgr, trigger_pub

def get_imu(data, the_imu):
    the_imu.pitch = data.pitch
    the_imu.yaw = data.yaw
    the_imu.roll = data.roll

def get_ft(data, the_ft):
    the_ft.fx = data.force[0]
    the_ft.fy = data.force[1]
    the_ft.fz = data.force[2]

    the_ft.tx = data.torque[0]
    the_ft.ty = data.torque[1]
    the_ft.tz = data.torque[2]

global count
count = 0
def get_js_and_control(data, args):

    demo = args[0]
    left_arm_pub = args[1]
    right_arm_pub = args[2]

    demo.js = data
    msgl, msgr, trigger = demo.move_arms_with_ft()

    global count
    count += 1
    if count != 10:
        return
    count = 0

    if trigger == True:
        left_arm_pub.publish(msgl)
        right_arm_pub.publish(msgr)




if __name__ == '__main__':

    left_arm_c_name = "left_arm_controller"
    right_arm_c_name = "right_arm_controller"
    head_c_name = "head_controller"
    waist_c_name = "waist_controller"
    legs_c_name = "arobot_trajectory_controller"

    left_arm_pub = rospy.Publisher('/arobot/%s/command' % (left_arm_c_name), JointTrajectory,queue_size=1000)
    right_arm_pub = rospy.Publisher('/arobot/%s/command' % (right_arm_c_name), JointTrajectory,queue_size=1000)
    head_pub = rospy.Publisher('/arobot/%s/command' % (head_c_name), JointTrajectory,queue_size=1000)
    waist_pub = rospy.Publisher('/arobot/%s/command' % (waist_c_name), JointTrajectory,queue_size=1000)
    legs_pub = rospy.Publisher('/arobot/%s/command' % (legs_c_name), JointTrajectory,queue_size=1000)


    rospy.init_node('sensor_ft_py', anonymous = False)

    demo = robotDemo()

    rospy.sleep(3)

    # rospy.Subscriber("/arobot/joint_states", JointState, echo)

    rospy.Subscriber("/arobot/l_foot_ft", ForceSix, get_ft, demo.l_ft_)
    rospy.Subscriber("/arobot/r_foot_ft", ForceSix, get_ft, demo.r_ft_)
    rospy.Subscriber("/arobot/body_nine_axis", ImuBasic, get_imu, demo.imu_)
    rospy.Subscriber("/arobot/joint_states", JointState, get_js_and_control, (demo, left_arm_pub, right_arm_pub))

    rospy.spin()



