#include <boost/date_time.hpp>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <math.h>
#include <string.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <iostream>

#include <arobot_whisper.h>
#include <arobot_kdl.h>

// rand function
double fRand(double min, double max)
{
  double f = (double)rand() / RAND_MAX;
  return min + f * (max - min);
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "control_arobot_kdl_demo");
	ros::NodeHandle nh("~");
	WhisperManager control(1000); // Set the control manager
	control.InitPublishers(nh);

	// Get the info from lanuch files.
	int num_samples;
	std::string chain_start, chain_end;
	double timeout;

	nh.param("chain_start", chain_start, std::string(""));
	nh.param("chain_end", chain_end, std::string(""));

	if (chain_start=="" || chain_end=="") {
		ROS_FATAL("Missing chain info in launch file");
		exit (-1);
	}

	nh.param("timeout", timeout, 0.005);

	AROBOT_KDL::ArobotKDL akdl(chain_start, chain_end, "urdf_param", timeout);


	KDL::Chain my_chain = akdl.getKDLChain();
	// Create Nominal chain configuration midway between all joint limits
	KDL::JntArray nominal(my_chain.getNrOfJoints());
	KDL::JntArray ll = akdl.getLowerLimits();
	KDL::JntArray ul = akdl.getTheUpperLimits();


	for (uint j = 0; j < nominal.data.size(); j++) {
		nominal(j) = (ll(j) + ul(j)) / 2.0;
	}

	// Basic infomation
	KDL::Frame end_effector_pose; // The desired axis.
	
	KDL::JntArray q(my_chain.getNrOfJoints()); // The result.

	for (uint j=0; j<ll.data.size(); j++) {
		q(j)=fRand(ll(j), ul(j));
	} // random of the result.

	KDL::JntArray resJnt = q; // The real result.
	KDL::JntArray result; // The calculated result.

	// FK
	akdl.JntToCart(q, end_effector_pose);

	ROS_INFO_STREAM("The desired axis is: "<<end_effector_pose.p[0]<<'\t'<<end_effector_pose.p[1]<<'\t'<<end_effector_pose.p[2]);
	
	// IK

	int rc=akdl.CartToJnt(q,end_effector_pose,result);

	if (rc>=0)
	{
		ROS_INFO_STREAM("The calculated result is: "<<result(0)<<'\t'<<result(1)<<'\t'<<result(2)<<'\t'<<result(3)<<'\t'<<result(4)<<'\t'<<result(5));
		ROS_INFO_STREAM("The real result is: "<<resJnt(0)<<'\t'<<resJnt(1)<<'\t'<<resJnt(2)<<'\t'<<resJnt(3)<<'\t'<<resJnt(4)<<'\t'<<resJnt(5));
	}
	else 
		ROS_INFO_STREAM("No solution with the KDL solution.");

	
	//publish the node
	
	// enum JointID {L_THIGHT_ROLL, L_THIGH_YAW, L_THIGH_PITCH, L_CALF_PITCH, L_FOOT_PITCH, L_FOOT_YAW, R_THIGHT_ROLL, R_THIGH_YAW, R_THIGH_PITCH, R_CALF_PITCH, R_FOOT_PITCH, R_FOOT_YAW, JOINT_NUM};
	
	ros::Duration(1).sleep();
	control.PubOnePosition(L_THIGH_ROLL, (double)result(5));
	control.PubOnePosition(L_THIGH_YAW, (double)result(4));
	control.PubOnePosition(L_THIGH_PITCH, (double)result(3));
	control.PubOnePosition(L_CALF_PITCH, (double)result(2));
	control.PubOnePosition(L_FOOT_PITCH, (double)result(1));
	control.PubOnePosition(L_FOOT_YAW, (double)result(0));

	control.ShutdownPublishers();
	return true;
}
