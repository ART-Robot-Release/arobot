# 关节/传感器数据读写 #

关节数据的操作，在仿真和实体环境中保持一致。（每一次控制周期细节的读写方式会有差异）

传感器数据的读写，在仿真和实机中略有不同。也可以相互转换。

## 仿真 ##

###　关节　###
Gazebo的仿真环境中关节的控制，正如[Manual](../README.md)所说。
其数据的获取，通过joint_state_controller发布。在xxx.yaml中会定义state_controller。

```
  # Publish all joint states -----------------------------------
  joint_state_controller:
    type: joint_state_controller/JointStateController
    publish_rate: 1000  

```

在Launch文件中，比如`load_trajectory_controller.launch`会使用`robot_state_publisher`来获取joint_states，
然后通过一系列坐标转换最终转换成robot_state_publisher，这个状态一般会被RVIZ订阅显示机器人。


```
<?xml version="1.0"?>
<launch>

  <!-- load the controllers -->
    <node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="false"
    output="screen" ns="/arobot" args="arobot_trajectory_controller joint_state_controller" />


  <!-- convert joint states to TF transforms for rviz, etc -->
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"
	  respawn="false" output="screen">
	  <remap from="/joint_states" to="/arobot/joint_states/" />
  </node>

</launch>
```

对于ROS-Controller的使用可以参考[http://wiki.ros.org/ros_control](http://wiki.ros.org/ros_control)官方wiki。

### 传感器　###

#### 力/力矩传感器和IMU ####
传感器数据的读取，跟controller如出一辙. 比如ft_sensor 

六维力传感器描述。

```
    <gazebo>
    	<plugin name="ft_sensor" filename="libgazebo_ros_ft_sensor.so">
			<robotNamespace>/arobot</robotNamespace>
    		<updateRate>10</updateRate>
    		<topicName>/arobot/ft_sensor_left</topicName>
   		 	<jointName>l_ankle_roll</jointName>
    	</plugin>
    </gazebo>
```

可以订阅/arobot/ft_sensor_left即可以获得六维力传感器的数据了。其数据格式可以定义在`src/robot_control/core/arobot_msgs/msg/ForceSix.msg`下
```
# This is a message to hold data from a six-axis force sensor

Header header

float64[3] force # Force about x, y, z axes

float64[3] torque # Torque about x, y, z axes
```

对于Message的应用，请参考[ROS官方的Message](http://wiki.ros.org/msg)手册。


#### 其他传感器 ####

- 里程计，Gazebo中可以使用（实机中没有）

发布base_link的程信息，主要用于RVIZ同步Gazebo中机器人的运动，效果是在RVIZ的机器人会产生移动。

- 视觉

虽然描述了视觉传感器，但只是一个简单的示例，仿真时请参照自己的需求进行调整。
 
----------

## 实机 ##

#### 关节 ####
实机的关节控制和读取与仿真完全没有差距。因此可以很容易的进行算法适配。

但是实机会有一些特殊的功能：
1. 实体机器可以微调每一次周期中的关节信息，请参考`arobot_hw/src/controller_main.cpp`

```
    robot->read(ros::Time::now(), ros::Duration(1.0/PUB_RATE)); // 得到硬件数据
	cm->update(ros::Time::now(), ros::Duration(1.0/PUB_RATE), false);　// 数据处理
	robot->write(ros::Time::now(), ros::Duration(1.0/PUB_RATE)); //　写入硬件数据

```

数据的更改和处理，可以在cm->update和read和write的间隔中处理。在此处的建议是，如果是数据处理和分析，可以在读写间处理。如果要对数据进行调整，建议到使用的controller中更改其update操作，否则可能会造成数据的不一致。由于ARTrobot现在只提供了ros默认提供的控制器，因此需要根据自己的情形参考使用。

2. 实体机器默认提供了数据库（实现在src/robot_control/core/database）的功能，为了实时的分析，显示以及记录数据，我们提供了基于[Redis](https://redis.io/)的内存数据库读写参考`arobot_hw/src/controller_main.cpp`中的案例。

```
	DataQueue *datas = DataQueue::getInstance();
	datas->setup(50);

	ExtDatabase *db =  ExtDatabaseFactory::createDB("redis");
	if(!db->connect("127.0.0.1", 6379))
    	ROS_WARN_STREAM_NAMED("MAIN", "redis connection failed.");
```

上面的代码会初始化一个内部和外部数据库，内部数据库仅仅是一个C++标准的deque队列，用来高速方便的控制内处理。外部数据库现在可支持基于Redis的数据库，我们采用一个单例的方式（请参考设计模式）来实现，将数据全部转换成Jason格式，以字符串的方式存储（可以容易的被前端以及脚本文件获取显示分析）。

```
	DataFrame &frame = datas->createDataFrame();
	frame.sendData = robot->data_send;
	frame.receiveData = robot->data_receive;
	frame.setFrameID(id++);
	if(!db->insertDataFrame("hi", frame))
    	ROS_WARN_STREAM_NAMED("MAIN", "redis insert failed.");
```

简单的插入操作如上。对于Redis数据库的读取使用，请参考其[官方](https://redis.io/)文档。

#### 传感器 ####

传感器的使用上，可以直接在 robot->data_send 以及 robot->data_receive 中获取。
如果要模拟成ros的节点，请参考`arobot_hw/src/controller_main.cpp`可以在每一次的周期循环中，通过ros的消息机制发送从硬件读取上来的信息。

```
	ros::Publisher l_ft_pub ;
	ros::Publisher r_ft_pub ;
	
	ros::Publisher nine_axis_pub ;

	........ // 读取数据

	l_ft_pub = nh.advertise<arobot_msgs::ForceSix>("l_foot_ft", 1000);
	r_ft_pub = nh.advertise<arobot_msgs::ForceSix>("r_foot_ft", 1000);

	nine_axis_pub = nh.advertise<arobot_msgs::ImuBasic>("body_nine_axis", 1000);
```
> 注：在新的版本中，会考虑在实机也以传感器插件的形式提供操作。
