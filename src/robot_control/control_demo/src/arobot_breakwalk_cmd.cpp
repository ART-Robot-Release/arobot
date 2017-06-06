#include <arobot_trans.h>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <control_msgs/QueryTrajectoryState.h>
#include <fstream>

#include <kdl/frames_io.hpp>

using std::string;

namespace
{
	double time_count = 0;
	double l_max_hip = 0;
	double l_min_hip = 0;

	double r_max_hip = 0;
	double r_min_hip = 0;
}
// rand function
double fRand(double min, double max)
{
  double f = (double)rand() / RAND_MAX;
  return min + f * (max - min);
}

KDL::JntArray getNomailJnt(arobot_limb& limb)
{
		KDL::JntArray nominal(limb.q.rows());
		KDL::JntArray ll = limb.ll;
		KDL::JntArray ul = limb.ul;

		ROS_INFO_ONCE_NAMED("arobot_trans_test", "Chain [%s] initialize the joint val.", limb.getTheName().c_str());

		for (uint j = 0; j < nominal.data.size(); j++) {
			nominal(j) = (ll(j) + ul(j)) / 2.0;
			ROS_INFO_ONCE_NAMED("arobot_trans_test","[%s] : joint[%d] = %f ", limb.getTheName().c_str(), j, nominal(j));
		}
		return nominal;
}

KDL::JntArray getZeroJnt(arobot_limb& limb)
{
		KDL::JntArray nominal(limb.q.rows());
		for (uint j = 0; j < nominal.data.size(); j++)
			nominal(j) = 0;
		return nominal;
}

KDL::JntArray getZeroJnt(const unsigned int n)
{
		KDL::JntArray nominal(n);
		for (uint j = 0; j != nominal.data.size(); j++)
			nominal(j) = 0.0;
		return nominal;
}


trajectory_msgs::JointTrajectoryPoint get_a_point(void)
{
    trajectory_msgs::JointTrajectoryPoint point;
	unsigned int n_joints;

    n_joints = (26);

    point.positions.resize(n_joints, 0.0);
    point.velocities.resize(n_joints, 0.0);
	point.accelerations.resize(n_joints, 0.0);

	return point;
}

bool knee_up(arobot_trans& trans, std::vector<trajectory_msgs::JointTrajectoryPoint>& points)
{
	trajectory_msgs::JointTrajectoryPoint point = get_a_point();
	time_count += 4.0;
	point.time_from_start = ros::Duration(time_count);
	points.push_back(point);

	return true;
}


bool zero_position(std::vector<trajectory_msgs::JointTrajectoryPoint>& points)
{
	time_count = 4.0;
	trajectory_msgs::JointTrajectoryPoint point = get_a_point();
	point.time_from_start = ros::Duration(time_count);
	points.push_back(point);

	return true;
}
bool init_kdl(arobot_trans& trans)
{
	// Basic infomation
	KDL::Frame l_end_effector_pose; // The desired axis.
	KDL::Frame r_end_effector_pose; // The desired axis.

	// get the limb from the name
	arobot_limb l_foot_limb;
	arobot_limb r_foot_limb;

	trans.getTheLimb("left_foot", l_foot_limb);
	trans.getTheLimb("right_foot", r_foot_limb);
	ROS_INFO_STREAM("The length of l_foot: "<<l_foot_limb.q.rows());
	ROS_INFO_STREAM("The length of r_foot: "<<r_foot_limb.q.rows());

	l_foot_limb.q = getZeroJnt(6);
	r_foot_limb.q = getZeroJnt(6);

	// FK zero
	trans.setTheJnt("left_foot", l_foot_limb.q);
	int ok = trans.JntToCart("left_foot");
	if(ok < 0)
		return false;
	trans.setTheJnt("right_foot", r_foot_limb.q);
	ok = trans.JntToCart("right_foot");
	if(ok < 0)
		return false;

	// Show
	trans.getThePose("left_foot",l_end_effector_pose);
	ROS_INFO_STREAM("The desired axis of left_foot is: "<<l_end_effector_pose.p[0]<<'\t'<<l_end_effector_pose.p[1]<<'\t'<<l_end_effector_pose.p[2]);
	trans.getThePose("right_foot",r_end_effector_pose);
	ROS_INFO_STREAM("The desired axis of right_foot is: "<<r_end_effector_pose.p[0]<<'\t'<<r_end_effector_pose.p[1]<<'\t'<<r_end_effector_pose.p[2]);

	// FK knee down
	l_foot_limb.q(2) = -0.45; l_foot_limb.q(3) = 0.9; l_foot_limb.q(4) = -0.45;
	r_foot_limb.q(2) = -0.45; r_foot_limb.q(3) = 0.9; r_foot_limb.q(4) = -0.45;

	KDL::JntArray resJntL = l_foot_limb.q;
	KDL::JntArray resJntR = r_foot_limb.q;

	trans.setTheJnt("left_foot", l_foot_limb.q);
	ok = trans.JntToCart("left_foot");
	if(ok < 0)
		return false;
	trans.setTheJnt("right_foot", r_foot_limb.q);
	ok = trans.JntToCart("right_foot");
	if(ok < 0)
		return false;
	// Show
	trans.getThePose("left_foot",l_end_effector_pose);
	ROS_INFO_STREAM("The desired axis of left_foot is: "<<l_end_effector_pose.p[0]<<'\t'<<l_end_effector_pose.p[1]<<'\t'<<l_end_effector_pose.p[2]);
	trans.getThePose("right_foot",r_end_effector_pose);
	ROS_INFO_STREAM("The desired axis of right_foot is: "<<r_end_effector_pose.p[0]<<'\t'<<r_end_effector_pose.p[1]<<'\t'<<r_end_effector_pose.p[2]);

	return true;
}

bool one_frame_foot(arobot_trans& trans, KDL::Frame& l_end_effector_pose, KDL::Frame& r_end_effector_pose,
        std::vector<trajectory_msgs::JointTrajectoryPoint>& points, double point_dur)
{
	// get the limb from the name
	KDL::JntArray resJntL;
	KDL::JntArray resJntR;


	// left
	int ok = trans.CartToJnt("left_foot", l_end_effector_pose);
	ROS_INFO_STREAM("The axis left_foot is: "<<l_end_effector_pose);
	ROS_INFO_STREAM("The axis right_foot is: "<<r_end_effector_pose);

	if(ok < 0)
		return false;
	trans.getTheJnt("left_foot", resJntL);

	// right
	ok = trans.CartToJnt("right_foot", r_end_effector_pose);
	if(ok < 0)
		return false;
	trans.getTheJnt("right_foot", resJntR);

	l_max_hip = resJntL(1) > l_max_hip ?  resJntL(1) : l_max_hip;
	l_min_hip = resJntL(1) < l_min_hip ?  resJntL(1) : l_min_hip;
	r_max_hip = resJntR(1) > r_max_hip ? resJntR(1) : r_max_hip;
	r_min_hip = resJntR(1) < r_min_hip ? resJntR(1) : r_min_hip;

	trajectory_msgs::JointTrajectoryPoint point = get_a_point();

	point.positions[2] = resJntL(0);point.positions[3] = resJntL(1);point.positions[4] = resJntL(2);point.positions[5] = resJntL(3);point.positions[7] = resJntL(4);point.positions[6] = resJntL(5);
	point.positions[8] = resJntR(0);point.positions[9] = resJntR(1);point.positions[10] = resJntR(2);point.positions[11] = resJntR(3);point.positions[13] = resJntR(4);point.positions[12] = resJntR(5);
    // raise arm
    point.positions[14] = -0.6;
    point.positions[19] = -0.6;
    point.positions[17] = -0.6;
    point.positions[22] = -0.6;



	point.time_from_start = ros::Duration(point_dur);
	points.push_back(point);

	return true;
}

bool knee_down(std::vector<trajectory_msgs::JointTrajectoryPoint>& points)
{
	trajectory_msgs::JointTrajectoryPoint point = get_a_point();

	point.positions[2] = 0;
	point.positions[3] = 0;
	point.positions[4] = -0.35;
	point.positions[5] = 0.7;
	point.positions[7] = -0.35;
	point.positions[6] = 0;

	point.positions[8] = 0;
	point.positions[9] = 0;
	point.positions[10] = -0.35;
	point.positions[11] = 0.7;
	point.positions[13] = -0.35;
	point.positions[12] = 0;

    // raise arm
    point.positions[14] = -0.6;
    point.positions[19] = -0.6;
    point.positions[17] = -0.6;
    point.positions[22] = -0.6;


	double tcount = 4.0;
	point.time_from_start = ros::Duration(time_count);
	points.push_back(point);
}

void load_foot_points(const std::string &filename,
					  arobot_trans &trans,
					  std::vector<trajectory_msgs::JointTrajectoryPoint> &points) {
	std::ifstream infile(filename);
	if (!infile.is_open()) {
		ROS_ERROR("please change the default path where contains data.txt, can not open it");
		exit(0);
	}

	double ctr_fre;
	std::string line;
	std::getline(infile, line);
	std::istringstream iss1(line);
	iss1 >> ctr_fre; // read control frequency

	double rotate_rad;
	std::getline(infile, line);
	std::istringstream iss2(line);
	iss2 >> rotate_rad; // read rotate rad
	KDL::Frame ident_l = KDL::Frame::Identity();
	KDL::Frame ident_r = KDL::Frame::Identity();

	KDL::Rotation rr(cos(rotate_rad), 0, sin(rotate_rad), 0, 1, 0,
			-sin(rotate_rad), 0.000000, cos(rotate_rad));

	double right_x, right_y, right_z, left_x, left_y, left_z;
	// std::getline(infile, line);
	//  std::istringstream init_line(line);
	//  init_line >> left_x >> left_y >> left_z >> right_x >> right_y >> right_z;

	trans.getThePose("right_foot", ident_r);
	trans.getThePose("left_foot", ident_l);
	KDL::Rotation rr_base_r = ident_r.M;
	KDL::Rotation rr_base_l = ident_l.M;

	double tcount = 0.5;
	ros::Duration(1).sleep();
	while (std::getline(infile, line)) {
		tcount += 1.0/ctr_fre;
		std::istringstream iss(line);
		iss >> left_x >> left_y >> left_z >> right_x >> right_y >> right_z;

	//	trans.getThePose("right_foot", ident_r);
		ident_r.p[0] = -1*right_x;
		ident_r.p[1] = right_y;
		ident_r.p[2] = right_z;

//		trans.getThePose("left_foot", ident_l);
		ident_l.p[0] = -1*left_x;
		ident_l.p[1] = left_y;
		ident_l.p[2] = left_z;

		// -0.244788	0.0173959	0.498422 -0.117556	0.172662	0.482339

		// -0.0240008	0.0310599	-0.247422
		if(!one_frame_foot(trans, ident_l, ident_r, points, tcount))
		{
			ROS_INFO_STREAM("Ik failed");
			// exit(0);
		}
	}
}

/* main */
int main (int argc, char *argv[])
{
	time_count = 0.0;
	ros::init(argc, argv, "walk_test");
	ros::NodeHandle nh("~");
	ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>("/arobot/arobot_trajectory_controller/command", 1000);
	arobot_trans trans(nh);

	if(trans.load())
	{
		ROS_INFO_NAMED("walk_test", "Configuration is okay");
	}
	else
	{
		ROS_FATAL_NAMED("walk_test", "Configuration is failed");
	}

	ros::Duration(2.0).sleep();

	// show the name of the chains
	const std::vector<arobot_limb>& theLimbs = trans.getLimbsList();
	for(auto i : theLimbs)
	{
		ROS_INFO_STREAM(i.getTheName().c_str());
	}

	if(!init_kdl(trans))
		ROS_INFO_STREAM("Knee down failed");

	std::vector<std::string> joint_names;
	joint_names.resize(26);
	joint_names[0] = "waist_pitch";
	joint_names[1] = "waist_yaw";
	joint_names[2] = "l_hip_yaw";
	joint_names[3] = "l_hip_roll";
	joint_names[4] = "l_hip_pitch";
	joint_names[5] = "l_knee_pitch";
	joint_names[7] = "l_ankle_pitch";
	joint_names[6] = "l_ankle_roll";
	joint_names[8] = "r_hip_yaw";
	joint_names[9] = "r_hip_roll";
	joint_names[10] = "r_hip_pitch";
	joint_names[11] = "r_knee_pitch";
	joint_names[13] = "r_ankle_pitch";
	joint_names[12] = "r_ankle_roll";
	joint_names[14] = "l_shoulder_pitch";
	joint_names[15] = "l_shoulder_roll";
	joint_names[16] = "l_elbow_yaw";
	joint_names[17] = "l_elbow_pitch";
	joint_names[18] = "l_wrist_yaw";
	joint_names[19] = "r_shoulder_pitch";
	joint_names[20] = "r_shoulder_roll";
	joint_names[21] = "r_elbow_yaw";
	joint_names[22] = "r_elbow_pitch";
	joint_names[23] = "r_wrist_yaw";
	joint_names[24] = "neck_yaw";
	joint_names[25] = "neck_pitch";


	// init the rostopic
	std::vector<trajectory_msgs::JointTrajectoryPoint> init_pos;
	trajectory_msgs::JointTrajectory u_traj;
	u_traj.joint_names = joint_names;
	if(!zero_position(init_pos))
		ROS_INFO_STREAM("Zero position failed");
	u_traj.points = init_pos;
	pub.publish(u_traj);
	// TODO: iks
	std::vector<trajectory_msgs::JointTrajectoryPoint> d_pos;
	trajectory_msgs::JointTrajectory d_traj;
	d_traj.joint_names = joint_names;
	if(!knee_down(d_pos))
		ROS_INFO_STREAM("Zero position failed");
	d_traj.points = d_pos;


	std::vector<trajectory_msgs::JointTrajectoryPoint> a_points, b_points, c_points, e_points, f_points;
	trajectory_msgs::JointTrajectory a_traj, b_traj, c_traj, e_traj, f_traj;
	a_traj.joint_names = joint_names;
	b_traj.joint_names = joint_names;
	c_traj.joint_names = joint_names;
	e_traj.joint_names = joint_names;
	f_traj.joint_names = joint_names;

	std::string a_fname(ros::package::getPath("control_demo")+"/src/testcases/data0.txt");
	std::string b_fname(ros::package::getPath("control_demo")+"/src/testcases/data1.txt");
	std::string c_fname(ros::package::getPath("control_demo")+"/src/testcases/data2.txt");
	std::string e_fname(ros::package::getPath("control_demo")+"/src/testcases/leftdata.txt");
	std::string f_fname(ros::package::getPath("control_demo")+"/src/testcases/rightdata.txt");

	load_foot_points(a_fname, trans, a_points);
	load_foot_points(b_fname, trans, b_points);
	load_foot_points(c_fname, trans, c_points);
	load_foot_points(e_fname, trans, e_points);
	load_foot_points(f_fname, trans, f_points);

	a_traj.points = a_points;
	b_traj.points = b_points;
	c_traj.points = c_points;
	e_traj.points = e_points;
	f_traj.points = f_points;


	bool is_up = true;
    while(ros::ok()){
	    std::cout << ">>> ";
		char cmd;
		std::cin >> cmd;
		if ('0' == cmd) {
			if (is_up) {
				std::cout << "standing" << std::endl;
				continue;
			}
			std::cout << "data0" << std::endl;
			pub.publish(a_traj);
		} else if ('1' == cmd) {
			if (is_up) {
				std::cout << "standing" << std::endl;
				continue;
			}
			std::cout << "data1" << std::endl;
			pub.publish(b_traj);
		} else if ('2' == cmd) {
			if (is_up) {
				std::cout << "standing" << std::endl;
				continue;
			}
			std::cout << "data2" << std::endl;
			pub.publish(c_traj);
		} else if ('3' == cmd) {
			if (is_up) {
				std::cout << "standing" << std::endl;
				continue;
			}
			std::cout << "leftdata" << std::endl;
			pub.publish(e_traj);
		} else if ('4' == cmd) {
			if (is_up) {
				std::cout << "standing" << std::endl;
				continue;
			}
			std::cout << "rightdata" << std::endl;
			pub.publish(f_traj);
		} else if ('u' == cmd) {
			std::cout << "stand up" << std::endl;
			pub.publish(u_traj);
			is_up = true;
		} else if ('d' == cmd) {
			std::cout << "knee down" << std::endl;
			pub.publish(d_traj);
			is_up = false;
		} else if ('q' == cmd) {
			break;
		}
    }

}

