# Manual #

ART-1 robot manual.

ART-1参考应用手册。

## 系统环境 ##

系统：ubuntu 16.04（推荐）


## 安装 ##

在源代码中运行项目目录下的安装脚本

	./install.sh

## 代码结构 ##
```
 ./doc --- Some simple script and docs.
 	./robot_control  --- The control of the robot.
		./common	 --- The common functions.
		./scripts    --- useful stripts.
		./control_demo	--- Simple demos.
		./libary     --- The libary of the robot control app.
		./core 		 --- The core files of robot.
			./arobot_hw		--- robot hardware functions.
			./arobot_msgs	--- robot messages.
			./database		--- The database function.
   ./robot_description 	--- The descriprtion of the robot (static).
   ./robot_gazebo 	--- The gazebo of the robot.
   ./gazebo_ros_pkgs	--- gazebo ros 插件
```

## 使用简介 ##

编译源代码，在项目根目录下（与src同级）

	catkin_make -Decat=OFF  

设置环境变量
	
	source devel/setup.bash

### 运行RVIZ显示机器人 ###

*在rviz中展示模型* 其中各个角度可以通过robot_state_publisher的工具（默认会打开）更改。

    roslaunch robot_description robot_rviz.launch

*在rviz中更新接受到的模型数据* ，在此功能中，rviz会接受由仿真器或者实际机器人反馈的信息，显示机器人的状态。

    roslaunch robot_description robot_rviz_run.launch

打开这个命令，你会惊奇的发现RVIZ会提示找不到link的相关错误，机器人也无法显示。这是因为，RVIZ并没有收到任何robot_state的message信息。
那么之前的launch为什么可以打开？如果有印象，之前的robot_rviz.launch会打开一个控制关节角度的小面板。这个小面板就是这种工具。

```
	<node name="joint_state_publisher" 
	pkg="joint_state_publisher"
	type="joint_state_publisher" />
```
那么_run的文件该如何才能显示呢，实际上，在我们真实运行的过程中，会运行一个不断对硬件读-改-写的过程。
```
roslaunch arobot_hw controller_main_demo.launch
```
如果没有实体机器的信息，这个读取数据都是原始0位置。RVIZ会收到这些数据，保持显示在机器人的初始位置。



### 仿真机器人环境 ###

我们默认使用*[gazebo](http://gazebosim.org/)*来仿真机器人模型。Gazebo会读取ARTrobot的描述文件，并且通过gazebo_ros_pkgs插件来与ros进行交互。（注：在下一代的ARTrobot中，我们会对这个插件进行改进）

#### 打开gazebo仿真环境（无控制版本） ####

    roslaunch robot_gazebo robot_display.launch

在这个版本中，机器人并没有收到任何控制器的控制。如果是第一次打开Gazebo，Gazebo将会从官方网站上跟新一些世界基础模型。如果网络不佳或者受到防火墙影响，将会需要等待几分钟的时间（首次打开建议在翻墙环境中）。
    
#### 打开gazebo仿真环境 (单关节控制版本) ####

    roslaunch robot_gazebo robot_gazebo.launch

在这版本中，机器人的关节控制器是独立的，各个关节可以独立控制。关节控制器的配置文件置于src/robot_control/core/arobot_hw/config/robot_ros_controller.yaml
```
arobot:
  # Publish all joint states -----------------------------------
  joint_state_controller:
    type: joint_state_controller/JointStateController
    publish_rate: 1000  
  

  waist_pitch_position_controller:
    type: effort_controllers/JointPositionController
    joint: waist_pitch
    pid: {p: 20000.0, i: 0.01, d: 0.0}



  waist_yaw_position_controller:
    type: effort_controllers/JointPositionController
    joint: waist_yaw
    pid: {p: 10000.0, i: 0.01, d: 0.0}

 # left leg

  l_hip_yaw_position_controller:
    type: effort_controllers/JointPositionController
    joint: l_hip_yaw
    pid: {p: 10000.0, i: 0.01, d: 0.0}
..............
```

使用方法可以参考[Gazebo与ros通信教程](http://gazebosim.org/tutorials?cat=connect_ros)其中一个简单的命令展示如下：

	rostopic pub -1 /arobot/waist_yaw_position_controller/command std_msgs/Float64 "data: 1.5"

ROS所提供的Controller_manager功能将会接收由rostopic发布的指令，更改waist_yaw的关节为1.5rad。

#### 打开gazebo仿真环境 (trajectory控制器版本) ####

	roslaunch robot_gazebo robot_gazebo_trajectory.launch 

在这版本中，机器人的关节控制器是统一规划到的[轨迹控制器](http://wiki.ros.org/joint_trajectory_controller)之中，所有关节将会按照一个位移点一个位移点的添加移动。控制器的配置文件置于src/robot_control/core/arobot_hw/config/robot_trajectory_controller.yaml

```	
arobot:
  # Publish all joint states -----------------------------------
  joint_state_controller:
    type: joint_state_controller/JointStateController
    publish_rate: 1000  

  arobot_trajectory_controller:
    allow_partial_joints_goal: true
    type: effort_controllers/JointTrajectoryController
    joints:
        - waist_pitch
        - waist_yaw
        # left leg
        - l_hip_yaw
        - l_hip_roll
        - l_hip_pitch
        - l_knee_pitch
        - l_ankle_pitch
        - l_ankle_roll
        # right leg 
        - r_hip_yaw
        - r_hip_roll
        - r_hip_pitch
        - r_knee_pitch
        - r_ankle_pitch
        - r_ankle_roll
        # left arm
        - l_shoulder_pitch
        - l_shoulder_roll
        - l_elbow_yaw
        - l_elbow_pitch
        - l_wrist_yaw
        # right arm
        - r_shoulder_pitch
        - r_shoulder_roll
        - r_elbow_yaw
        - r_elbow_pitch
        - r_wrist_yaw
        # head
        - neck_yaw
        - neck_pitch
    gains:
        waist_pitch:  {p: 20000.0, i: 0.01, d: 0.0, i_clamp: 1}
        waist_yaw: {p: 20000.0, i: 0.01, d: 0.0, i_clamp: 1}
        # left leg
.........
```

ROS所提供的Controller_manager功能将会接收由topic/action发送的指令控制机器人，相关的例子可以参考control_demo下的control_trajectory_pub.py

	rosrun control_demo control_trajectory_pub.py -d arm

移动双臂例子(其他支持的命令wave, twist...)

```
		.........
        point_start = JointTrajectoryPoint()
        point_start.positions = [0 for i in range(26)]
        point_start.time_from_start = rospy.Duration(1)
        msg.points.append(point_start)

        point1 = JointTrajectoryPoint()
        point1.positions = [0 for i in range(26)]
        self.set_the_rad(point1, 30, 'l_shoulder_pitch')
        self.set_the_rad(point1, -30, 'r_shoulder_pitch')
        msg.points.append(point1)
		.........
		pub = rospy.Publisher('/arobot/arobot_trajectory_controller/command', JointTrajectory,queue_size=1000)
		.........
```
    
#### 使用Server + Client模式启动GAZEBO ####
[Gazebo支持Server/Client模式](http://gazebosim.org/tutorials?tut=components&cat=get_started)这样在小团队中，只需要一台性能较高的仿真机器即可。
- server:

```
roslaunch robot_gazebo robot_gazebo.launch
```
- client:

ROS的设置(假定仿真机为master)：

*需要在你的bashrc下面设置*
```
function get_ip_address { ifconfig | fgrep -v 127.0.0.1 | fgrep 'Mask:255.255.255.0' | egrep -o 'addr:[^ ]*' | sed 's/^.*://'; }

export ROS_IP=$( get_ip_address )
export ROS_MASTER_URI=http://192.168.1.200:11311 # 请更改ip地址
```

gazebo的设置：

*需要在你的bashrc下面设置*
```
export GAZEBO_IP=$( get_ip_address )
export GAZEBO_MASTER_URI=192.168.1.200:12321 # 请更改ip和端口
```
> WARNING: client需要有跟server一致的mesh文件
    
#### 简单的demo ####

ARTrobot的设计可以支持控制行为无缝工作于仿真环境和实体机器。

想要测试这些功能你可以做以下的几种不同操作

1. 单独打开Rviz观测

		roslaunch robot_description robot_rviz_run.lanuch

2. （同时）打开Gazebo操控机器人
	
		roslaunch robot_gazebo robot_gazebo_trajectory.launch 

3. （同时）打开实体机器操控机器人
	
		roslaunch arobot_hw controller_main.launch （此操作需要实体机器人）
	
	或者
			
		roslaunch arobot_hw trajectory_controller_main.launch （此操作需要实体机器人）
		

**给机器人发送简单的动作指令：**

其中control_trajectory_pub.py是使用Python编写的简单的动作示例。执行的例子在src/robot_control/control_demo/src下面

	rosrun control_demo control_trajectory_pub.py -d "命令" 

其中命令有 "arm,wave,twist" 等，可以在脚本的最后找到这些命令。

**机器人简单行走案例：**
机器人简单行走的离线数据保存在src/robot_control/control_demo/src/testcases/test_data.txt测试数据中。
运行这些数据只需要运行（如果没有运行roscore请先执行`roscore`）
```
roslaunch arobot_hw load_param.launch
rosrun control_demo arobot_walk
```
其中load_param.launch会设置一些运动学解算的参数。执行的例子在src/robot_control/control_demo/src下面。

> 请注意，ARTrobot的代码大多基于C++11/14以及Python，这也是ROS所推荐的。


### 其他参考文档 ###

- [URDF文件简介](./docs/urdf.md)
- [关节/传感器数据读写](./docs/joint_sensor.md)
- [传感器使用案例](./docs/sensor_demo.md)
- [库文件简介](./docs/lib.md)
- [MoveIt使用案例](./docs/move_it.md)
- [ART-1操作手册](./docs/artrobot_1.md)
- [RealSense使用](./docs/real_sense.md)
- [Record功能](./docs/record.md)
