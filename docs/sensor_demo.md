# 传感器数据使用demo #

该文档是建立于
[关节/传感器数据读写](joint_sensor.md)之上，实现了几种获取传感器的案例。


## 通过ROS中的topic来获取 ##

我们在`src/robot_control/core/arobot_hw/src/controller_main.cpp`中发布了imu和六维力传感器的数据。

```
	l_ft_pub = nh.advertise<arobot_msgs::ForceSix>("l_foot_ft", 1000);
	r_ft_pub = nh.advertise<arobot_msgs::ForceSix>("r_foot_ft", 1000);

	nine_axis_pub = nh.advertise<arobot_msgs::ImuBasic>("body_nine_axis", 1000);

    ......

	auto pub_sensors = [&robot]()
	{

		ros::Rate rate(SENSOR_PUB_RATE);

		while(ros::ok())
		{
			l_ft_pub.publish(get_ft_msg(robot.l_foot_ft));
			r_ft_pub.publish(get_ft_msg(robot.r_foot_ft));

			nine_axis_pub.publish(get_nine_axis_msg(robot.body_nine_axis));

		}

		rate.sleep();
	};

	std::thread ts(pub_sensors);
```

因此只要订阅相关的topic就可以获取到传感器数据，在这里我们提供两个例子供给参考

首先我们提供了一组将双臂，头部，腰部，下肢的controller的launch。

```
arobot:
  # Publish all joint states -----------------------------------
  joint_state_controller:
    type: joint_state_controller/JointStateController
    publish_rate: 1000

  arobot_trajectory_controller:
    allow_partial_joints_goal: true
    type: position_controllers/JointTrajectoryController
    joints:
        # left leg
        - l_hip_yaw
        - l_hip_roll
        - l_hip_pitch
        - l_knee_pitch
        - l_ankle_roll
        - l_ankle_pitch
        # right leg
        - r_hip_yaw
        - r_hip_roll
        - r_hip_pitch
        - r_knee_pitch
        - r_ankle_roll
        - r_ankle_pitch

  waist_controller:
    allow_partial_joints_goal: true
    type: position_controllers/JointTrajectoryController
    joints:
        - waist_pitch
        - waist_yaw


  head_controller:
    allow_partial_joints_goal: true
    type: position_controllers/JointTrajectoryController
    joints:
        # head
        - neck_yaw
        - neck_pitch

  left_arm_controller:
      allow_partial_joints_goal: true
      type: position_controllers/JointTrajectoryController
      joints:
          # left arm
          - l_shoulder_pitch
          - l_shoulder_roll
          - l_elbow_yaw
          - l_elbow_pitch
          - l_wrist_yaw

  right_arm_controller:
      allow_partial_joints_goal: true
      type: position_controllers/JointTrajectoryController
      joints:
          # right arm
          - r_shoulder_pitch
          - r_shoulder_roll
          - r_elbow_yaw
          - r_elbow_pitch
          - r_wrist_yaw
```

只需要运行

    roslaunch arobot_hw body_controller_main.launch

即可。

`src/robot_control/control_demo/src/sensor_control.py` 提供了一个IMU的案例。


这个案例中拿到IMU的信息，控制双臂进行摆动保持平衡。

```
    ......

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

```

`src/robot_control/control_demo/src/sensor_ft.py` 提供了一个使用六维力的案例。


这个案例中拿到FT的信息，在单脚支持的使用摆动双臂。

```
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
```
