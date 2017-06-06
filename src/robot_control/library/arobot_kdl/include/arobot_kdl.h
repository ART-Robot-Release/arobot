/**
* @file arobot_kdl.h
* @detail The header file of arobot_kdl
* @author IcePie
* @email sunxiao.gin@gmail.com
* @version 0.0.1
* @date 2016-07-28 13:10:36
**/


#ifndef AROBOT_KL_H
#define AROBOT_KL_H

#include <trac_ik/trac_ik.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/tree.hpp>

#include <ros/ros.h>

#include <memory> // for auto ptr / C++ 11
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

namespace AROBOT_KDL {

  enum SolveType { Speed, Distance, Manip1, Manip2 }; // come from the trac ik 

  class ArobotKDL
  {
	  public:
		  ArobotKDL(const KDL::Chain& _chain, const KDL::JntArray& _q_min, const KDL::JntArray& _q_max, double _maxtime=0.005, double _eps=1e-2, SolveType _type=Speed);
		  // find "urdf_xml" if not (find "URDF_param") 
		  ArobotKDL(const std::string& base_link, const std::string& tip_link, const std::string& URDF_param="/robot_description", double _maxtime=0.005, double _eps=1e-5, SolveType _type=Speed);
		  
		  //  ~ArobotKDL();

		  // Inverse Kinematics.
		  int CartToJnt(const KDL::JntArray &q_init, const KDL::Frame &p_in, KDL::JntArray &q_out, const KDL::Twist& bounds=KDL::Twist::Zero());
		  // Forward Kinematics.
		  int JntToCart(const KDL::JntArray &q_in, KDL::Frame &p_out, int segmentNr = -1);

   
		  inline KDL::Chain getKDLChain(void) { return this->chain; }
		  inline KDL::JntArray getTheUpperLimits(void) { return this->ll; }
		  inline KDL::JntArray getLowerLimits(void) { return this->ul; }

		  inline void setSolveType(SolveType _type) 
		  {
			  this->solvetype = _type;
		  }

		  inline bool initOk(void)
		  {
			  return initialized;
		  }


	  private:
		  
		  bool initialized;
		  KDL::Chain chain;
		  KDL::JntArray ll, ul; /**< lower joint limits, upper joint limits */
		  double eps;
		  double maxtime;
		  SolveType solvetype;

		  TRAC_IK::TRAC_IK tracik_solver; /**< The trac ik class */
		  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fksolver; /**< The fw solver */
		  // KDL::ChainIkSolverVel_pinv vik_solver;

		  void initialize(); /**< The function of initialization which for setting up some info after the chain getted. */
  };


	struct COM
	{
		COM(){ mass = 0; vaild = false; };

		KDL::Vector pos;
		double mass;
		bool vaild;
	};

	class ArobotTree
	{
	public:
		bool loadKDLModel(const urdf::Model & robot_model_)
		{
			if (!kdl_parser::treeFromUrdfModel(robot_model_, kdl_tree_)) {
				ROS_ERROR("Could not initialize tree object");
				return false;
			}

			setFootname("l_foot_link", "r_foot_link");
            return true;
		}

		bool loadKDLModel(const std::string & robot_model_)
		{
			if (!kdl_parser::treeFromString(robot_model_, kdl_tree_)) {
				ROS_ERROR("Could not initialize tree object");
				return false;
			}
			setFootname("l_foot_link", "r_foot_link");
            return true;
		}

		void setFootname(const std::string& lfoot, const std::string& rfoot)
		{
			lfoot_link_name_ = lfoot;
			rfoot_link_name_ = rfoot;
		}

		COM computeCOM(const std::map<std::string, double>& joint_positions); //, KDL::Frame& tf_right_foot, KDL::Frame& tf_left_foot);
        COM computeCOM(const std::map<std::string, double>& joint_positions, KDL::Frame& tf_right_foot, KDL::Frame& tf_left_foot);
		void computeCOMRecurs(const KDL::SegmentMap::const_iterator& current_seg, const std::map<std::string, double>& joint_positions,
						 const KDL::Frame& tf, KDL::Frame& tf_right_foot, KDL::Frame& tf_left_foot, double& m, KDL::Vector& com);



	private:
		KDL::Tree kdl_tree_;
		std::string lfoot_link_name_;
		std::string rfoot_link_name_;

	};



}

#endif // AROBOT_KL_H

