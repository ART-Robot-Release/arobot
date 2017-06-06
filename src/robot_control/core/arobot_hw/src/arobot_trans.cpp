#include <arobot_trans.h>

bool arobot_limb::setupChain(const string& start, const string& end, const std::string& URDF_param, double _maxtime, double _eps, SolveType _type )
{
	akdl_ptr = std::make_shared<AROBOT_KDL::ArobotKDL>(start, end, URDF_param, _maxtime, _eps, _type);
	// set up the joint array
	KDL::Chain my_chain = akdl_ptr->getKDLChain();
	q.resize(my_chain.getNrOfJoints());
	ll = akdl_ptr->getLowerLimits();
	ul = akdl_ptr->getTheUpperLimits();
	assert(q.rows() == ll.rows() && q.rows() == ul.rows());
	return akdl_ptr->initOk();
}

bool arobot_limb::setupChain(const std::string& URDF_param, double _maxtime, double _eps, SolveType _type)
{
	akdl_ptr = std::make_shared<AROBOT_KDL::ArobotKDL>(_start, _end, URDF_param, _maxtime, _eps, _type);
	// set up the joint array
	KDL::Chain my_chain = akdl_ptr->getKDLChain();
	q.resize(my_chain.getNrOfJoints());
	ll = akdl_ptr->getLowerLimits();
	ul = akdl_ptr->getTheUpperLimits();
	assert(q.rows() == ll.rows() && q.rows() == ul.rows());
	return akdl_ptr->initOk();
}


// ------------------------------------------------------------------------------------------------------------------------------------------------------------


bool arobot_trans::load(std::string param_name)
{
	using std::string;
	std::string search_param_name;
	std::vector<string> limps_list;
	if (_nh.searchParam("robot_tree", search_param_name))
	{
		ROS_INFO_NAMED("arobot_trans", "arobot hardware is initialing the robot tree for kdl from [%s]", search_param_name.c_str());
		_nh.getParam(search_param_name, limps_list);
		_limbs.resize(limps_list.size());
	}
	else
	{
		ROS_FATAL_NAMED("arobot_trans", "robot_tree can not be found." );
		return false;
	}

	int i = 0;
	for(auto val : limps_list)
	{
		std::vector<string> chain_tag;
		if (_nh.searchParam(val, search_param_name))
		{
			_nh.getParam(search_param_name, chain_tag);
			_limbs[i] = arobot_limb(val.c_str(), chain_tag[0].c_str(), chain_tag[1].c_str());
			_limbs[i].setupChain();
			++i;
			ROS_INFO_NAMED("arobot_trans", "chain [%s] - start : [%s] end [%s] is set", val.c_str(), chain_tag[0].c_str(), chain_tag[1].c_str());
		}
		else
		{
			ROS_FATAL_NAMED("arobot_trans", "chain info can not be found." );
			return false;
		}

	}

	std::string urdf_string;

	// search and wait for robot_description on param server
	while (urdf_string.empty())
	{
		std::string search_param_name;
		if (_nh.searchParam(param_name, search_param_name))
		{
			_nh.getParam(search_param_name, urdf_string);
		}
		else
		{
			_nh.getParam(param_name, urdf_string);
		}
		usleep(100000);
	}
	ROS_DEBUG_STREAM_NAMED("arobot_trans", "Recieved urdf from param server, parsing...");

	if(!_robot_tree.loadKDLModel(urdf_string))
	{
		ROS_FATAL_NAMED("arobot_trans","Could not find robot description parameter.");
		return false;
	}
	return true;
}


bool arobot_trans::getTheLimb(const string& chain_name, arobot_limb& limb)
{
	std::vector<arobot_limb>::iterator res = find(_limbs.begin(), _limbs.end(), chain_name);
	if(res == _limbs.end())
	{
		ROS_FATAL_NAMED("arobot_trans", "The [%s] chain can not be found.", chain_name.c_str());
		return false;
	}
	limb = *res;
	return true;
}

bool arobot_trans::setTheJnt(const string& chain_name, const KDL::JntArray &q)
{
	std::vector<arobot_limb>::iterator res = find(_limbs.begin(), _limbs.end(), chain_name);
	if(res == _limbs.end())
	{
		ROS_FATAL_NAMED("arobot_trans", "The [%s] chain can not be found.", chain_name.c_str());
		return false;
	}
	res->setTheJnt(q);
	return true;
}

bool arobot_trans::getTheJnt(const string& chain_name, KDL::JntArray& q)
{
	std::vector<arobot_limb>::iterator res = find(_limbs.begin(), _limbs.end(), chain_name);
	if(res == _limbs.end())
	{
		ROS_FATAL_NAMED("arobot_trans", "The [%s] chain can not be found.", chain_name.c_str());
		return false;
	}
	q = res->getTheJnt();
	return true; 
}


bool arobot_trans::setThePose(const string& chain_name, const KDL::Frame &p)
{
	std::vector<arobot_limb>::iterator res = find(_limbs.begin(), _limbs.end(), chain_name);
	if(res == _limbs.end())
	{
		ROS_FATAL_NAMED("arobot_trans", "The [%s] chain can not be found.", chain_name.c_str());
		return false;
	}
	res->setThePose(p);
	return true;
}

bool arobot_trans::getThePose(const string& chain_name, KDL::Frame &p)
{
	std::vector<arobot_limb>::iterator res = find(_limbs.begin(), _limbs.end(), chain_name);
	if(res == _limbs.end())
	{
		ROS_FATAL_NAMED("arobot_trans", "The [%s] chain can not be found.", chain_name.c_str());
		return false;
	}
	p = res->getThePose();
	return true;
}



// Inverse Kinematics.
int arobot_trans::CartToJnt(const string& chain_name, const KDL::JntArray &q_init, const KDL::Frame &p_in, KDL::JntArray &q_out, const KDL::Twist& bounds)
{
	std::vector<arobot_limb>::iterator res = find(_limbs.begin(), _limbs.end(), chain_name);
	if(res == _limbs.end())
	{
		ROS_FATAL_NAMED("arobot_trans", "The [%s] chain can not be found.", chain_name.c_str());
		return -1;
	}

	int ret = res->akdl_ptr->CartToJnt(q_init, p_in, q_out, bounds);
	if(ret < 0)
	{
		ROS_FATAL_NAMED("arobot_trans", "The IK of [%s] failed.", chain_name.c_str());
		return ret;
	}
	res->setTheJnt(q_out);
	res->setThePose(p_in);
	return ret;
}

int arobot_trans::CartToJnt(const string& chain_name, const KDL::Frame &p_in, const KDL::Twist& bounds)
{
	std::vector<arobot_limb>::iterator res = find(_limbs.begin(), _limbs.end(), chain_name);
	if(res == _limbs.end())
	{
		ROS_FATAL_NAMED("arobot_trans", "The [%s] chain can not be found.", chain_name.c_str());
		return -1;
	}

	KDL::JntArray q_out;
	int ret = res->akdl_ptr->CartToJnt(res->getTheJnt(), p_in, q_out, bounds);

	if(ret < 0)
	{
		ROS_FATAL_NAMED("arobot_trans", "The IK of [%s] failed.", chain_name.c_str());
		return ret;
	}
	res->setTheJnt(q_out);
	res->setThePose(p_in);
	return ret;
}

int arobot_trans::CartToJnt(const string& chain_name, const KDL::Twist& bounds)
{
	std::vector<arobot_limb>::iterator res = find(_limbs.begin(), _limbs.end(), chain_name);
	if(res == _limbs.end())
	{
		ROS_FATAL_NAMED("arobot_trans", "The [%s] chain can not be found.", chain_name.c_str());
		return -1;
	}

	KDL::JntArray q_out;
	int ret = res->akdl_ptr->CartToJnt(res->getTheJnt(), res->getThePose(), q_out, bounds);

	if(ret < 0)
	{
		ROS_FATAL_NAMED("arobot_trans", "The IK of [%s] failed.", chain_name.c_str());
		return ret;
	}
	res->setTheJnt(q_out);
	return ret;
}


// Forward Kinematics.
int arobot_trans::JntToCart(const string& chain_name, KDL::JntArray &q_in, KDL::Frame &p_out, int segmentNr)
{
	std::vector<arobot_limb>::iterator res = find(_limbs.begin(), _limbs.end(), chain_name);
	if(res == _limbs.end())
	{
		ROS_FATAL_NAMED("arobot_trans", "The [%s] chain can not be found.", chain_name.c_str());
		return -1;
	}
	int ret = res->akdl_ptr->JntToCart(q_in, p_out, segmentNr);
	if(ret < 0)
	{
		ROS_FATAL_NAMED("arobot_trans", "The FK of [%s] failed.", chain_name.c_str());
		return ret;
	}
	res->setTheJnt(q_in);
	res->setThePose(p_out);
	return ret;
}

int arobot_trans::JntToCart(const string& chain_name, KDL::JntArray &q_in, int segmentNr)
{
	std::vector<arobot_limb>::iterator res = find(_limbs.begin(), _limbs.end(), chain_name);
	if(res == _limbs.end())
	{
		ROS_FATAL_NAMED("arobot_trans", "The [%s] chain can not be found.", chain_name.c_str());
		return -1;
	}

	KDL::Frame p_out;

	int ret = res->akdl_ptr->JntToCart(q_in, p_out, segmentNr);
	if(ret < 0)
	{
		ROS_FATAL_NAMED("arobot_trans", "The FK of [%s] failed.", chain_name.c_str());
		return ret;
	}
	res->setTheJnt(q_in);
	res->setThePose(p_out);
	return ret;
}

int arobot_trans::JntToCart(const string& chain_name, int segmentNr)
{
	std::vector<arobot_limb>::iterator res = find(_limbs.begin(), _limbs.end(), chain_name);
	if(res == _limbs.end())
	{
		ROS_FATAL_NAMED("arobot_trans", "The [%s] chain can not be found.", chain_name.c_str());
		return -1;
	}

	KDL::Frame p_out;
	int ret = res->akdl_ptr->JntToCart(res->getTheJnt(), p_out, segmentNr);

	if(ret < 0)
	{
		ROS_FATAL_NAMED("arobot_trans", "The FK of [%s] failed.", chain_name.c_str());
		return ret;
	}
	res->setThePose(p_out);
	return ret;
}

// get the com
AROBOT_KDL::COM arobot_trans::computeCOM(sensor_msgs::JointState & jointState)
{
	std::map<std::string, double> map;

	assert(jointState.name.size() == jointState.position.size());
	for( int i = 0; i != jointState.name.size(); ++i)
		map[jointState.name[i]] = jointState.position[i];

	return this->_robot_tree.computeCOM(map);
}

AROBOT_KDL::COM arobot_trans::computeCOM(sensor_msgs::JointState & jointState, KDL::Frame & tf_right_foot, KDL::Frame& tf_left_foot)
{
    std::map<std::string, double> map;

    assert(jointState.name.size() == jointState.position.size());
    for( int i = 0; i != jointState.name.size(); ++i)
        map[jointState.name[i]] = jointState.position[i];

    return this->_robot_tree.computeCOM(map, tf_right_foot, tf_left_foot);
}

