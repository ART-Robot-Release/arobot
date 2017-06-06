//
// Created by han on 16-12-29.
//

#ifndef ROS_MASTER_NODE_ECAT_INIT_H
#define ROS_MASTER_NODE_ECAT_INIT_H

#include <memory>
#include <unordered_map>

#include "ros/ros.h"

#include "AngleToNumber.h"

#include "art_robot_ecat.h"

void init_ecat(std::unordered_map<int, std::shared_ptr<AngleToNumber>>& angle_cal_dict);

void init_ecat_from_ros(ros::NodeHandle& nh, std::unordered_map<int, std::shared_ptr<AngleToNumber>>& angle_cal_dict, std::unordered_map<int, std::string>& motor_to_joint_name);

#endif //ROS_MASTER_NODE_ECAT_INIT_H
