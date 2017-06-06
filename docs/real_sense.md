# Art-1 realsense#

ART-1 的视觉传感器使用了Intel的realSense系列中的200。

其接入到了顶层机器，即IP为100的机器上。在使用的时候有2种方式可以获取200的数据：
1. 通过OpenCV或者其他工具直接从200中读取图像数据。
2. 通过ROS提供的工具读取数据。


其中通过ROS的节点非常便捷，我们主要采用这种方式，但是由于图像流不比一般的message。因此另ROS的通信系统承担较大的负载，采用网络集群的方式，
图像的效果和延迟，也会受限于网络性能。

## 在顶层机器上打开RealSense ##

根据[ROS-Wiki](http://wiki.ros.org/realsense_camera)中的描述，在机器上运行

    roslaunch realsense_camera r200_nodelet_default.launch
    
## 在其他机器上获取图像 ##

可以使用rqt_image_view查看图像

    rqt_image_view
    
使用Rviz查看图像

    roslaunch robot_description robot_rviz_run.launch

之后点击add，选择image，在选择想要使用的topic即可。
> 如果没有运行机器人的controller发布joint_state信息，是无法看到机器人机体的显示的。


