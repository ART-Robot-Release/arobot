function get_ip_address { ifconfig | fgrep -v 127.0.0.1 | fgrep 'Mask:255.255.255.0' | egrep -o 'addr:[^ ]*' | sed 's/^.*://'; }        

export ROS_IP=$( get_ip_address ) 
export ROS_MASTER_URI=http://192.168.8.100:11311

source ../devel/setup.bash
