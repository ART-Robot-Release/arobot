# Artrobot中文使用手册 ART-Robot 1 #

**注意**

> 不慎进入机器人运动范围内或者与机器人发生接触，都有可能引发人身伤害事故。当异常时，请立即按下紧急停机键。紧急停机键位于机器人后背靠上红色按钮。


## 参数配置 ##

- 重量：50kg
- 高度：135cm
- 控制器：Inter Core i7 Processor*1 + Inter Core i5 Processor*1
- 对外传感器：
1. F/T Sensor*2
2. Camera Inter RealSense*1
3. IMU+Temperature*1

- 通信：用户接口：Wifi/以太网
- 自由度(26个)：
1. 头部：2个
2. 腰部：2个
3. 手臂：5*2个
4. 腿部：6*2个
- 电池：锂电池，48.1V，10ah，持续工作4小时
- 开发环境：
1. OS:Linux Ubuntu 16.04 64bit
2. SW:ROS Kinetic
3. Compiler & Script: GNU C++/Python

## 操作教程 ##

**默认电脑已经安装Linux Ubuntu 16.04 64bit（推荐）**

### 打开第一个终端，远程登录 ###
1. 按下机器人开机按钮（后背中部），按钮中部蓝色指示灯亮起，等待路由器开机，这个过程可能需要几分钟，直道搜索到无线网络humanoid；
2. 计算机搜索并连接无线wifi，帐号：humanoid 密码：artrobot;
3. 打开一个终端（快捷键Ctrl+Alt+T）；
4. 在终端里输入：`ssh artrobot@192.168.8.100` (第一次连接需要输入yes )  密码是：a
5. 在终端里输入：`roscore`
6. 此终端输入完毕，放置一边不再操作；

### 打开第二个终端，再次远程登录 ###
1. 重新打开一个终端（快捷键Ctrl+Alt+T）；
2. 在终端里输入：`ssh han@192.168.8.101`，(第一次连接需要输入yes )    密码是：a
3. 在终端里输入：`sudo /etc/init.d/ethercat start`   密码是：a
4. 在终端里输入：`ls`，
5. 在终端里输入：`cd arobot`
6. 在终端里输入：`source devel/setup.bash`
7. 再次按下机器人开机按钮（后背中部)
8. 在终端里输入：`roslaunch arobot_hw trajectory_controller_main.launch`

### 打开第三个终端，再次远程登录 ###
1. 重新打开一个终端（快捷键Ctrl+Alt+T
2. 在终端里输入：ssh han@192.168.8.101 密码是：a
3. 在终端里输入：`cd arobot`
4. 在终端里输入：`source devel/setup.bash`
5. 在终端里输入：`rosrun control_demo control_trajectory_pub.py -d "arm"` 

### 打开第四个终端，同步仿真 ###
1. 重新打开一个终端（快捷键Ctrl+Alt+T）；
2. 在终端里输入：`cd arobot`
3. 在终端里输入：`cd docs`
4. 在终端里输入：`source debug.bash` 
5. 在终端里输入：`roslaunch robot_description robot_rviz_run.launch`
6. 重新在终端三里输入命令，即可实现实体与仿真同步。

### 关机 ###
1. 打开主控(101)的终端；
2. 在终端里输入：按快捷键Ctrl+C，结束当前运行
3. 在终端里输入：`sudo poweroff` 
4. 打开顶层机器(100)终端；
5. 在终端里输入：`sudo poweroff` 
6. 关闭电源。


## 电机/传感器控制服务 ##

基于ROS, ART-1提供了非常灵活的service。

当开机连接机器，各关节电机受控之后。(一般为执行了`roslaunch arobot_hw xxxx_main.launch`) 就会开启这些服务。

    rosservice list 
可以看到
```
/arobot/actuator_control
/arobot/actuators_detect
....
/arobot/drivers_detect
/arobot/master_controller_detect
/arobot/sensors_calibration
/arobot/sensors_detect

```

### 电机控制 ###

    rosservice call /arobot/actuator_control "command: 'xxx'"

我们现在支持4种指令 
    
    rosservice call /arobot/actuator_control "command: 'shutdown'"

Shutdown 可以关闭电机控制，表现的形式为电机会读取数据，但是不会受控制，可以随意挪动。
    
    rosservice call /arobot/actuator_control "command: 'poweron'"
    
Poweron 打开电机控制，电机会在当前位置受控停止。该行为只有在shutdown情况下可用。

    rosservice call /arobot/actuator_control "command: 'cal'"

Cal 标定，标定只有在 Poweron 和 Run 状态下可用，标定零点位置。

    rosservice call /arobot/actuator_control "command: 'start'"
    rosservice call /arobot/actuator_control "command: 'stop'"
        
Start 开启正常运作，开始默认状态， 需要注意的是，会恢复到之前运行状态设置的位置而非shutdown后的当前位置。
Stop  暂停当前电机，在Run状态下可以执行，一般和Start混合使用。



