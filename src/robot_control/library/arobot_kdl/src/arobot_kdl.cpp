/**
* @file arobot_kl.cpp
* @detail The source file of ArobotKDL
* @author IcePie
* @email sunxiao.gin@gmail.com
* @version 0.0.1
* @date 2016-07-28 13:09:33
**/


#include <arobot_kdl.h>

#include <iostream>

namespace AROBOT_KDL {

	// Constructor Function - 2
	ArobotKDL::ArobotKDL(const std::string& base_link, const std::string& tip_link, const std::string& URDF_param, double _maxtime, double _eps, SolveType _type):
		initialized(false),
		maxtime(_maxtime),
		eps(_eps),
		solvetype(_type),
		// This constructor parses the URDF loaded in rosparm urdf_param into the
		// needed KDL structures. We setup them with trac ik libary. (trac ik can getKDLLimits from urdf.)
		tracik_solver(base_link, tip_link, URDF_param, _maxtime, _eps, (TRAC_IK::SolveType)_type)
	{

		bool valid = this->tracik_solver.getKDLChain(this->chain);

		if (!valid) {
			ROS_ERROR("There was no valid KDL chain found");
			return;
		}
		valid = tracik_solver.getKDLLimits(this->ll,this->ul);

		if (!valid) {
			ROS_ERROR("There were no valid KDL joint limits found");
			return;
		}
		assert(this->chain.getNrOfJoints() == this->ll.data.size());
		assert(this->chain.getNrOfJoints() == this->ul.data.size());

		ROS_INFO ("Using %d joints",this->chain.getNrOfJoints());

		initialize();
	}

	// Constructor Function - 1
	ArobotKDL::ArobotKDL(const KDL::Chain& _chain, const KDL::JntArray& _q_min, const KDL::JntArray& _q_max, double _maxtime, double _eps, SolveType _type):
		initialized(false),
		chain(_chain),
		ll(_q_min),
		ul(_q_max),
		maxtime(_maxtime),
		eps(_eps),
		solvetype(_type),
		tracik_solver(_chain, _q_min, _q_max, _maxtime, _eps, (TRAC_IK::SolveType)_type)
	{
		initialize();
	}

	void ArobotKDL::initialize(void)
	{
		assert(chain.getNrOfJoints()==ll.data.size());
		assert(chain.getNrOfJoints()==ul.data.size());

		// setup some pointers
		this->fksolver.reset(new KDL::ChainFkSolverPos_recursive(this->chain)); 

		if(!this->fksolver)
		{
			ROS_ERROR(" The Foward Kinematics solver can not be initialized.");
			initialized = false;
			return;
		}
		initialized = true;
	}


	// Forward Kinematics.
	int ArobotKDL::JntToCart(const KDL::JntArray &q_in, KDL::Frame &p_out, int segmentNr)
	{
		if (!initialized) 
		{
			ROS_ERROR(" The ArobotKDL is not properly initialized with a valid chain or limits.");
			return -1;
		}

		// The KDL fk
		return this->fksolver->JntToCart(q_in, p_out, segmentNr);
	}


	int ArobotKDL::CartToJnt(const KDL::JntArray &q_init, const KDL::Frame &p_in, KDL::JntArray &q_out, const KDL::Twist& bounds)
	{
		if (!initialized) 
		{
			ROS_ERROR(" The ArobotKDL is not properly initialized with a valid chain or limits.");
			return -1;
		}
		// The trac_ik
		return this->tracik_solver.CartToJnt(q_init, p_in, q_out, bounds);
	}

	void ArobotTree::computeCOMRecurs(const KDL::SegmentMap::const_iterator& current_seg, const std::map<std::string, double>& joint_positions,
						  const KDL::Frame& tf, KDL::Frame& tf_right_foot, KDL::Frame& tf_left_foot, double& m, KDL::Vector& com) {

		double jnt_p = 0.0;

		if (current_seg->second.segment.getJoint().getType() != KDL::Joint::None){
			std::map<std::string, double>::const_iterator jnt = joint_positions.find(current_seg->second.segment.getJoint().getName());

			if (jnt == joint_positions.end()){
				ROS_WARN("Could not find joint %s of %s in joint positions. Aborting tree branch.", current_seg->second.segment.getJoint().getName().c_str(), current_seg->first.c_str());
				return;
			}
			jnt_p = jnt->second;
		}

		KDL::Frame current_frame = tf * current_seg->second.segment.pose(jnt_p);
		if (current_seg->first == lfoot_link_name_){
			tf_left_foot = current_frame;
			ROS_INFO("Left foot tip transform found");
		} else if (current_seg->first == rfoot_link_name_){
			tf_right_foot = current_frame;
			ROS_INFO("Right foot tip transform found");
		}


		KDL::Vector current_cog = current_seg->second.segment.getInertia().getCOG();
		double current_m = current_seg->second.segment.getInertia().getMass();


		com = com + current_m * (current_frame*current_cog);

		m += current_m;
		ROS_DEBUG("At link %s. local: %f / [%f %f %f]; global: %f / [%f %f %f]",current_seg->first.c_str(), current_m, current_cog.x(), current_cog.y(), current_cog.z(),
				  m, com.x(), com.y(), com.z());

		// TODO: separate recursive fct to create markers, callable on demand
//  if (current_m > 0.0){
//    visualization_msgs::Marker marker;
//    createCoGMarker(current_seg->first, "torso", 0.02, (current_frame*current_cog), marker);
//    com_vis_markers_.markers.push_back(marker);
//  }

		std::vector<KDL::SegmentMap::const_iterator >::const_iterator child_it;
		for (child_it = current_seg->second.children.begin(); child_it !=current_seg->second.children.end(); ++child_it){
			computeCOMRecurs(*child_it, joint_positions, current_frame, tf_right_foot, tf_left_foot, m, com);
		}
	}

	COM ArobotTree::computeCOM(const std::map<std::string, double>& joint_positions) // KDL::Frame& tf_right_foot, KDL::Frame& tf_left_foot)
	{
        KDL::Frame right_foot_tf = KDL::Frame::Identity();
        KDL::Frame left_foot_tf = KDL::Frame::Identity();
        return computeCOM(joint_positions, right_foot_tf, left_foot_tf);
	}

    COM ArobotTree::computeCOM(const std::map<std::string, double>& joint_positions, KDL::Frame& tf_right_foot, KDL::Frame& tf_left_foot)
    {
        double mass = 0.0;
        COM com;
        KDL::Frame ident = KDL::Frame::Identity();
        KDL::Frame transform = ident;
        KDL::Frame right_foot_tf = ident;
        KDL::Frame left_foot_tf = ident;

        computeCOMRecurs(kdl_tree_.getRootSegment(), joint_positions, transform, right_foot_tf, left_foot_tf, com.mass, com.pos);
        if (left_foot_tf == ident || right_foot_tf == ident){
            ROS_ERROR("Could not obtain left or right foot transforms");
            return com;
        }

        if (com.mass <= 0.0){
            ROS_WARN("Total mass is 0, no CoM possible.");
            return com;
        }

        com.pos = 1.0/com.mass * com.pos;
        ROS_DEBUG("Total mass: %f CoG: (%f %f %f)", com.mass, com.pos.x(), com.pos.y(), com.pos.z());
        com.vaild = true;

        // COM.setValue(com.x(), com.y(), com.z());
        tf_right_foot = right_foot_tf;
        tf_left_foot = left_foot_tf;
        // tf::transformKDLToTF(right_foot_tf, tf_right_foot);
        // tf::transformKDLToTF(left_foot_tf, tf_left_foot);
    }


} // end namespace AROBOT_KDL

