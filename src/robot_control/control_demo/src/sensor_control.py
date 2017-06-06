#!/usr/bin/env python
# -* - coding: UTF-8 -* -
# this demo need trajectory_controller_main.launch.
import rospy
import string
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from arobot_msgs.msg import ImuBasic
import copy
import sys, getopt


def angleToRad(angle):
    return angle*3.1416/180

class robotDemo:
    joint_names_legs = []
    joint_names_larm = []
    joint_names_rarm = []
    joint_names_waist = []
    joint_names_head = []
    joint_ids = {}
    count = 0
    ap = 0.0
    ar = 0.0

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

        self.count = 0
        self.ap = 0.0
        self.ar = 0.0

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

    def move_to_start(self):
        msg = self.get_msg_with_name()
        point_start = JointTrajectoryPoint()
        point_start.positions = [0 for i in range(26)]
        point_start.time_from_start = rospy.Duration(3)
        msg.points.append(point_start)

        return msg


    def adj_body(self,pitch, pitch_zero, roll, roll_zero):

        self.count += 1
        if self.count != 5:
            self.ap += pitch
            self.ar += roll
            return None, None
        self.count = 0
        self.ap = self.ap / 5.0
        self.ar = self.ar / 5.0

        msgl = self.get_msg_with_name('left_arm')
        msgr = self.get_msg_with_name('right_arm')

        point_start = JointTrajectoryPoint()
        point_start.positions = [0 for i in range(5)]
        # point_start.time_from_start = rospy.Duration(1)
        # msg.points.append(point_start)

        # point1 = JointTrajectoryPoint()
        point1 = copy.deepcopy(point_start)
        point2 = copy.deepcopy(point_start)
        point1.positions = [0 for i in range(5)]
        point2.positions = [0 for i in range(5)]

        self.set_the_rad(point1, self.ap*2.0 + pitch_zero, 'r_shoulder_pitch')
        self.set_the_rad(point2, self.ap*2.0+ pitch_zero, 'l_shoulder_pitch')
        self.set_the_rad(point1, self.ap + pitch_zero, 'r_elbow_pitch')
        self.set_the_rad(point2, self.ap + pitch_zero, 'l_elbow_pitch')

        self.set_the_rad(point1, self.ar*2.0 - roll_zero, 'r_shoulder_roll')
        self.set_the_rad(point2, self.ar*2.0 + roll_zero, 'l_shoulder_roll')

        point1.time_from_start = rospy.Duration(0.1)
        point2.time_from_start = rospy.Duration(0.1)

        msgl.points.append(point2)
        msgr.points.append(point1)

        return msgl, msgr


global js
js = JointState()
def echo(data):
    global js
    js = data
# print ("The data of joint is :\n")
# print (js)


global gpitch
global groll
gpitch = None
groll = None
def adj(data, args):

    global gpitch
    if gpitch == None:
        gpitch = data.pitch

    global groll
    if groll == None:
        groll = data.roll

    pub_larm = args[0]
    pub_rarm = args[1]
    demo = args[2]
    pz = args[3]
    rz = args[4]

    msgl, msgr = demo.adj_body(-1.0 * (data.pitch - gpitch) , pz,  -1.0*(data.roll - groll) , rz)
    if msgl != None and msgr != None :
        pub_larm.publish(msgl)
        pub_rarm.publish(msgr)






if __name__ == '__main__':

    pitch_zero = 0
    roll_zero = 0

    try:
        opts, args = getopt.getopt(sys.argv[1:],"p:r:",["pitch=", "roll="])
    except getopt.GetoptError:
        print '-p [pitch zero value]'
        print '-r [roll zero value]'
        sys.exit(2)

    for opt, arg in opts:
        if opt in ('-p', "--pitch"):
            pitch_zero = float(arg)
        if opt in ('-r', "--roll"):
            roll_zero = float(arg)


    # pub = rospy.Publisher('/arobot/arobot_trajectory_controller/command', JointTrajectory,queue_size=1000)

    rospy.init_node('sensor_control_py', anonymous = False)


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

    demo = robotDemo()

    rospy.sleep(3)

    # rospy.Subscriber("/arobot/joint_states", JointState, echo)
    rospy.Subscriber("/arobot/body_nine_axis", ImuBasic, adj, (left_arm_pub, right_arm_pub, demo, pitch_zero, roll_zero))
    rospy.Subscriber("/arobot/joint_states", JointState, echo)

    rospy.spin()



