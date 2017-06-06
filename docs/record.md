# Art-1 Record #

ART-1 关节记录功能。

该功能是为了实现简易示教而制作

## 前期准备 ##

请根据之前的文档，打开机器人并进入**正常工作**状态。

## 关闭机器人 ##

参考中文使用手册：
    
    rosservice call /arobot/actuator_control "command: 'shutdown'"

关闭电机控制系统。

## 开启记录节点 ##
    
    rosrun control_demo record_js.py

> 如果该节点无法接受指令打印，请在101的主控机器上开启该节点。

## 发送记录命令　##

    rostopic pub -1 /arobot/record_cmd std_msgs/String "data: ''"
    
现阶段，该功能不区分指令内容，每次的指令都会出发一次记录，打印在屏幕上

比如

```
time +=tparam
point= copy.deepcopy(point_start)
self.move_all(point, time , -0.00, -0.00, -0.92, 0.23, 0.00, -0.00, 0.00, -0.00, 0.09, -0.00, -0.00, 0.00, 0.00, 0.00, -0.00, -0.00, -0.00, 0.00, 0.05, 0.00, 0.00, 0.00, -0.00, -0.00, 0.01, 0.00)
msg.points.append(point)
```

这些点连接起来后，可以粘贴到`control_trajectory_pub.py`的新动作函数中，形成指令，注意，时间参数需要自己手动调节。


## TO DO LIST ##

- Bug : 非主控机器无法显示，待测试。
- Command需要增加内容，比如可以收集信息，直接生成函数等。
- 关节映射关系，是写死的，有跟发送指令不同步的可能性（如果有人改变了文档中的列表顺序）以后需要统一某位置或者用名字映射的方式存储。
