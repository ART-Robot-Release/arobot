//
// Created by xsun on 17-4-26.
//

#include <vector>
#include <string>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>

int main(int argc, char **argv)
{
    std::vector<std::string> joint_name_list;
    ros::init(argc, argv, "trajectory_send");
    ros::NodeHandle nh;

    ROS_INFO("trajectory_send_node start!!!");

    nh.getParam("trajectory_send_node/joints_names", joint_name_list);

    //for(auto &item : joint_name_list){
    //    ROS_INFO("%s", item.c_str());
    //}

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> move("/arobot/arobot_trajectory_controller/follow_joint_trajectory", true);
    move.waitForServer();

    ROS_INFO("Connected to server");

    trajectory_msgs::JointTrajectory traj;

    trajectory_msgs::JointTrajectoryPoint p;
    for(auto& item : joint_name_list){
        traj.joint_names.push_back(item);

        if(item == "l_shoulder_pitch" or item == "r_shoulder_pitch"){
            p.positions.push_back(1.57);
        }else{
            p.positions.push_back(0.0);
        }
        p.accelerations.push_back(0.0);
        p.velocities.push_back(0.0);
        p.effort.push_back(0.0);

    }

    p.time_from_start = ros::Duration(10.0);
    traj.points.push_back(p);

    trajectory_msgs::JointTrajectoryPoint pp;
    for(auto& item : joint_name_list){
        //traj.joint_names.push_back(item);

        pp.positions.push_back(0.0);
        pp.accelerations.push_back(0.0);
        pp.velocities.push_back(0.0);
        pp.effort.push_back(0.0);

    }

    pp.time_from_start = ros::Duration(20.0);

    traj.points.push_back(pp);


    traj.header.stamp = ros::Time::now();

    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = traj;
    move.sendGoal(goal);

    /*
    other method for wait
     */
    // while(ros::ok() && !move.waitForResult(ros::Duration(5.0)) {};

    bool finished_within_time = move.waitForResult(ros::Duration(120.0));
    if (!finished_within_time) {
        move.cancelGoal();
        ROS_INFO("Timed out achieving goal A");
    } else {
        actionlib::SimpleClientGoalState state = move.getState();
        bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
        if (success)
            ROS_INFO("Action finished: %s", state.toString().c_str());
        else {
            ROS_INFO("Action failed: %s", state.toString().c_str());
            ROS_WARN("Addition information: %s", state.text_.c_str());
        }
    }

    ros::shutdown();
}