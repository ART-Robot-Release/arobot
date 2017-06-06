# 仿真电脑
#         IP: 192.168.1.200
#         电脑名：artrobot-sim
#         用户名：artrobot 密码: a

# 系统安装在ssd上面，2TB的hdd挂载在/home/artrobot/Data 这个目录下

# ROS的设置(假定仿真机为master)：
# 需要在你的bashrc下面设置
function get_ip_address { ifconfig | fgrep -v 127.0.0.1 | fgrep 'Mask:255.255.255.0' | egrep -o 'addr:[^ ]*' | sed 's/^.*://'; }

export ROS_IP=$( get_ip_address )
export ROS_MASTER_URI=http://192.168.1.200:11311

# gazebo的设置：
# 需要在你的bashrc下面设置
export GAZEBO_IP=$( get_ip_address )
export GAZEBO_MASTER_URI=192.168.1.200:12321

# 运行gazebo的时候需要在仿真机上面开启gzserver  命令如同 gzserver -e dart 其中-e dart 表示使用dart这个物理引擎
# 然后在本机运行gzclient
