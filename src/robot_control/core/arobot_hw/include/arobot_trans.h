#ifndef AROBOT_TRANS_H
#define AROBOT_TRANS_H

// arobot libary
#include <arobot_kdl.h>
#include <arobot_hw.h>

// ros
#include <ros/ros.h>

// C++
#include <vector>
#include <algorithm>

using std::string;
using std::shared_ptr;
using AROBOT_KDL::SolveType;

class arobot_limb
{
	public:
		arobot_limb()
		{
			_name = "";
			_start = "";
			_end = "";
		}
		arobot_limb(string name, string start, string end)
		{
			_name = name;
			_start = start;
			_end = end;
		}

		bool operator== (const arobot_limb& b) const
		{
			return _name == b._name ? true : false;
		}

		bool operator== ( const string& name) const
		{
			return _name == name ? true : false;
		}

		bool setupChain(const string& start, const string& end, const std::string& URDF_param="/robot_description", double _maxtime=0.005, double _eps=1e-5, SolveType _type=AROBOT_KDL::Speed );


		bool setupChain(const std::string& URDF_param="/robot_description", double _maxtime=0.005, double _eps=1e-5, SolveType _type=AROBOT_KDL::Speed );


		void setThePose(const KDL::Frame& pose)
		{
			this->pose = pose;
		}
		KDL::Frame getThePose(void)
		{
			return this->pose;
		}

		void setTheJnt(const KDL::JntArray& jnt)
		{
			this->q = jnt;
		}
		KDL::JntArray getTheJnt(void)
		{
			return this->q;
		}

		string getTheName(void)
		{
			return _name;
		}

		shared_ptr<AROBOT_KDL::ArobotKDL> akdl_ptr;
		
		KDL::Frame pose; // The axis.
		KDL::JntArray q; // The Joint.
		KDL::JntArray ul; // The Joint upper limit.
		KDL::JntArray ll; // The Joint lower limit.

	private:
		string _name;
		string _start;
		string _end;
		
};

class arobot_trans
{
	public:
		arobot_trans(string nh_name)
		{
			_nh = ros::NodeHandle(nh_name);
		};

		arobot_trans(ros::NodeHandle nh):_nh(nh)
		{
		};
	

		bool load(std::string = "robot_description");

		const std::vector<arobot_limb>& getLimbsList(void)
		{
			return _limbs;
		}

		bool getTheLimb(const string& chain_name, arobot_limb& limb);

		// joints array
		bool setTheJnt(const string& name, const KDL::JntArray& q);
		bool getTheJnt(const string& name, KDL::JntArray& q);
		
		// pose
		bool setThePose(const string& name, const KDL::Frame& p);
		bool getThePose(const string& name, KDL::Frame& p);

	 	// Inverse Kinematics.
		int CartToJnt(const string& chain_name, const KDL::JntArray& q_init, const KDL::Frame& p_in, KDL::JntArray& q_out, const KDL::Twist& bounds=KDL::Twist::Zero());
		int CartToJnt(const string& chain_name, const KDL::Frame& p_in, const KDL::Twist& bounds=KDL::Twist::Zero());
		int CartToJnt(const string& chain_name, const KDL::Twist& bounds=KDL::Twist::Zero());
		
		// Forward Kinematics.
		int JntToCart(const string& chain_name, KDL::JntArray& q_in, KDL::Frame& p_out, int segmentNr = -1);
		int JntToCart(const string& chain_name, KDL::JntArray& q_in, int segmentNr = -1);
		int JntToCart(const string& chain_name, int segmentNr = -1);

		// get the com
		AROBOT_KDL::COM computeCOM(sensor_msgs::JointState & jointState); //, KDL::Frame & tf_right_foot, KDL::Frame& tf_left_foot);
        AROBOT_KDL::COM computeCOM(sensor_msgs::JointState & jointState, KDL::Frame & tf_right_foot, KDL::Frame& tf_left_foot);

	private:
		ros::NodeHandle _nh;
		std::vector<arobot_limb> _limbs;
		AROBOT_KDL::ArobotTree _robot_tree;
};

#endif // AROBOT_TRANS_H
