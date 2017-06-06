
function get_ip_address { ifconfig | fgrep -v 127.0.0.1 | fgrep 'Mask:255.255.255.0' | egrep -o 'addr:[^ ]*' | sed 's/^.*://'; }        

unset ROS_IP
unset ROS_MASTER_URI

source /opt/ros/kinetic/setup.bash
