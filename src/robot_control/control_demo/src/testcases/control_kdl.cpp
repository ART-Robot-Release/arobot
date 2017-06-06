#include <boost/date_time.hpp>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <math.h>
#include <string.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <iostream>

#include <arobot_whisper.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <urdf/model.h>

#include <boost/make_shared.hpp>
#include <limits>

// rand function
double fRand(double min, double max)
{
  double f = (double)rand() / RAND_MAX;
  return min + f * (max - min);
}

// Get the limit from the urdf file.
// Todo Get The limit from hw class.
bool getTheLimit(const std::string& xml_string, KDL::Chain chain, KDL::JntArray& lb, KDL::JntArray& ub)
{
    urdf::Model robot_model;

 	robot_model.initString(xml_string);
	
	std::cout<<"Reading joints and links from URDF"<<std::endl;

    std::vector<KDL::Segment> chain_segs = chain.segments;
    boost::shared_ptr<const urdf::Joint> joint;

	lb.resize(chain.getNrOfJoints());
	ub.resize(chain.getNrOfJoints());

	uint joint_num=0;
	for(unsigned int i = 0; i < chain_segs.size(); ++i) {
		joint = robot_model.getJoint(chain_segs[i].getJoint().getName());
		if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
			joint_num++;
			float lower, upper;
			int hasLimits;
			if ( joint->type != urdf::Joint::CONTINUOUS ) {
				if(joint->safety) {
					lower = std::max(joint->limits->lower, joint->safety->soft_lower_limit);
					upper = std::min(joint->limits->upper, joint->safety->soft_upper_limit);
				} else {
					lower = joint->limits->lower;
					upper = joint->limits->upper;
				}
				hasLimits = 1;
			}
			else {
				hasLimits = 0;
			}
			if(hasLimits) {
				lb(joint_num-1)=lower;
				ub(joint_num-1)=upper;
			}
			else {
				lb(joint_num-1)=std::numeric_limits<float>::lowest();
				ub(joint_num-1)=std::numeric_limits<float>::max();
			}
			ROS_INFO_STREAM("joint "<<joint->name<<" "<<lb(joint_num-1)<<" "<<ub(joint_num-1));
		}
	}
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "control_kdl_demo");
	ros::NodeHandle nh("~");
	WhisperManager control(1000); // Set the control manager
	control.InitPublishers(nh);
	// Call KDL parser
	KDL::Tree my_tree;
	std::string robot_des_string;
	nh.param("urdf_param", robot_des_string, std::string());
	// show the string

	std::cout<<"Parser the urdf file"<<std::endl;

	if(!kdl_parser::treeFromString(robot_des_string, my_tree))
	{
		ROS_ERROR("Failed to construct kdl tree.");
		return false;
	}

	std::cout<<"The parser worked well"<<std::endl;

	std::cout<<"Convert the kdl::tree to kdl::chain"<<std::endl;



	int num_samples;
	std::string chain_start, chain_end;
	double timeout;

	nh.param("chain_start", chain_start, std::string(""));
	nh.param("chain_end", chain_end, std::string(""));

	if (chain_start=="" || chain_end=="") {
		ROS_FATAL("Missing chain info in launch file");
		exit (-1);
	}

	KDL::Chain my_chain;
	if(!my_tree.getChain(chain_start, chain_end, my_chain)) {
		ROS_FATAL("Couldn't find chain %s to %s",chain_start.c_str(), chain_end.c_str());
		exit (-1);
	}

	nh.param("timeout", timeout, 0.005);
	std::cout<<"The chain convertion is okay : "<<my_chain.getNrOfJoints()<<std::endl;

	std::cout<<"Get the limit from the urdf string "<<std::endl;

	KDL::JntArray ll, ul;

	getTheLimit(robot_des_string, my_chain, ll, ul);

	std::cout<<" Start to caculate the inverse kinematics "<<std::endl;
	// Set up KDL IK
	double eps = 1e-5;
	KDL::ChainFkSolverPos_recursive fk_solver(my_chain); // Forward kin. solver
	KDL::ChainIkSolverVel_pinv vik_solver(my_chain); // PseudoInverse vel solver
	KDL::ChainIkSolverPos_NR_JL kdl_solver(my_chain,ll,ul,fk_solver, vik_solver, 1, eps); // Joint Limit Solver

	// Create Nominal chain configuration midway between all joint limits
	KDL::JntArray nominal(my_chain.getNrOfJoints());

	for (uint j = 0; j < nominal.data.size(); j++) {
		nominal(j) = (ll(j) + ul(j)) / 2.0;
	}

	KDL::Frame end_effector_pose; // The desired axis.
	KDL::JntArray q(my_chain.getNrOfJoints()); // The result.

	for (uint j=0; j<ll.data.size(); j++) {
		q(j)=fRand(ll(j), ul(j));
	} // random of the result.

	KDL::JntArray resJnt = q; // The result.
	fk_solver.JntToCart(q, end_effector_pose);

	ROS_INFO_STREAM("The desired axis is: "<<end_effector_pose.p[0]<<'\t'<<end_effector_pose.p[1]<<'\t'<<end_effector_pose.p[2]);

	boost::posix_time::ptime start_time;
	boost::posix_time::time_duration diff;

	KDL::JntArray result;

	int rc; 
	double total_time=0;
	double elapsed = 0;
	result=nominal; // start with nominal
	start_time = boost::posix_time::microsec_clock::local_time();
	do {
		q=result; // when iterating start with last solution
		rc=kdl_solver.CartToJnt(q,end_effector_pose,result);
		diff = boost::posix_time::microsec_clock::local_time() - start_time;
		elapsed = diff.total_nanoseconds() / 1e9;
	} while (rc < 0 && elapsed < timeout);
	total_time+=elapsed;

	if (rc>=0)
		ROS_INFO_STREAM("KDL with an average of "<<total_time<<" secs per sample");
	else 
		ROS_INFO_STREAM("No solution with the KDL solution.");

	ROS_INFO_STREAM("The calculated result is: "<<result(0)<<'\t'<<result(1)<<'\t'<<result(2)<<'\t'<<result(3)<<'\t'<<result(4)<<'\t'<<result(5));
	ROS_INFO_STREAM("The real result is: "<<resJnt(0)<<'\t'<<resJnt(1)<<'\t'<<resJnt(2)<<'\t'<<resJnt(3)<<'\t'<<resJnt(4)<<'\t'<<resJnt(5));

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
