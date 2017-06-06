#include <arobot_trans.h>

#include <ros/ros.h>

using std::string;
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

bool test_the_chain(arobot_trans& trans, const string chain_name)
{
	// Basic infomation
	KDL::Frame end_effector_pose; // The desired axis.
	
	// get the limb from the name
	arobot_limb the_limb;
	trans.getTheLimb(chain_name, the_limb);

	// generate a random joint list
	KDL::JntArray q(the_limb.q.rows()); // The result.

	for (uint j=0; j<the_limb.q.rows(); j++) {
		q(j)=fRand(the_limb.ll(j), the_limb.ul(j));
	} // random of the result.

	KDL::JntArray resJnt = q; // The real result.
	KDL::JntArray result; // The calculated result.

	// 3-1 FK
	int ok = trans.JntToCart(chain_name, q, end_effector_pose);
	if(ok < 0)
		return false;

	ROS_INFO_STREAM("The desired axis of "<<chain_name<< " is: "<<end_effector_pose.p[0]<<'\t'<<end_effector_pose.p[1]<<'\t'<<end_effector_pose.p[2]);
	
	// 3-2 IK
	trans.setTheJnt(chain_name, getNomailJnt(the_limb));
	ok = trans.CartToJnt(chain_name, end_effector_pose);
	if(ok < 0)
		return false;

	if(!trans.getTheJnt(chain_name, result))
		return false;

	// compare them
	ROS_INFO_STREAM("The calculated result is: ");
	for(int i = 0; i != result.rows(); i++)
		ROS_INFO_STREAM(result(i));

	ROS_INFO_STREAM("The real result is: ");
	for(int i = 0; i != resJnt.rows(); i++)
		ROS_INFO_STREAM(resJnt(i));
	
	return true;
}

/* main */
int main (int argc, char *argv[])
{
	ros::init(argc, argv, "control_trans_test");
	ros::NodeHandle nh("~");
	
	arobot_trans trans(nh);
	
	// 1. Test load()
	if(trans.load())
	{
		ROS_INFO_NAMED("arobot_trans_test", "Configuration is okay");
	}
	else
	{
		ROS_FATAL_NAMED("arobot_trans_test", "Configuration is failed");
	}

	// show the name of the chains
	const std::vector<arobot_limb>& theLimbs = trans.getLimbsList();
	for(auto i : theLimbs)
	{
		ROS_INFO_STREAM(i.getTheName().c_str());
	}
	
	// Test IK and FK
	// Create the random joint value.

	// Create Nominal chain configuration midway between all joint limits
	for(auto i : theLimbs)
	{
		// 2. set the init value
		trans.setTheJnt(i.getTheName(), getNomailJnt(i));

		// 3. test the chain
		if(test_the_chain(trans, i.getTheName()))
			ROS_INFO_NAMED("arobot_trans_test","FK + IK for [%s] is okay ", i.getTheName().c_str());
	}


    // test com
    sensor_msgs::JointState js;

    js.name.resize(26);
    js.position.resize(26);
    std::for_each(js.position.begin(), js.position.end(),[](double& ele){ ele = 0;});
    js.name[0] = "waist_pitch";
    js.name[1] = "waist_yaw";
    js.name[2] = "l_hip_yaw";
    js.name[3] = "l_hip_roll";
    js.name[4] = "l_hip_pitch";
    js.name[5] = "l_knee_pitch";
    js.name[7] = "l_ankle_pitch";
    js.name[6] = "l_ankle_roll";
    js.name[8] = "r_hip_yaw";
    js.name[9] = "r_hip_roll";
    js.name[10] = "r_hip_pitch";
    js.name[11] = "r_knee_pitch";
    js.name[13] = "r_ankle_pitch";
    js.name[12] = "r_ankle_roll";
    js.name[14] = "l_shoulder_pitch";
    js.name[15] = "l_shoulder_roll";
    js.name[16] = "l_elbow_yaw";
    js.name[17] = "l_elbow_pitch";
    js.name[18] = "l_wrist_yaw";
    js.name[19] = "r_shoulder_pitch";
    js.name[20] = "r_shoulder_roll";
    js.name[21] = "r_elbow_yaw";
    js.name[22] = "r_elbow_pitch";
    js.name[23] = "r_wrist_yaw";
    js.name[24] = "neck_yaw";
    js.name[25] = "neck_pitch";

    AROBOT_KDL::COM com = trans.computeCOM(js);

    if(com.vaild) {
        ROS_INFO_STREAM("com vector: [" << com.pos.x() <<", "<<com.pos.y() << ", "<<com.pos.z());
        ROS_INFO_STREAM("mass: " << com.mass);
    } else
        ROS_INFO_STREAM("com cal failed");

    KDL::Frame right_foot_tf = KDL::Frame::Identity();
    KDL::Frame left_foot_tf = KDL::Frame::Identity();

    com = trans.computeCOM(js, right_foot_tf, left_foot_tf);

    if(com.vaild) {
        ROS_INFO_STREAM("com vector: [" << com.pos.x() <<", "<<com.pos.y() << ", "<<com.pos.z());
        ROS_INFO_STREAM("mass: " << com.mass);
        ROS_INFO_STREAM("left: " << left_foot_tf.p[0] << "\t" << left_foot_tf.p[1] << "\t" << left_foot_tf.p[2]);
        ROS_INFO_STREAM("right: " << right_foot_tf.p[0] << "\t" << right_foot_tf.p[1] << "\t" << right_foot_tf.p[2]);
    } else
        ROS_INFO_STREAM("com cal failed");




}

