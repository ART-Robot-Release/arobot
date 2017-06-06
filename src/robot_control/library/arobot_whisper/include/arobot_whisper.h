/**
* @file control_manager.h
* @author Galaxy2416
* @email sunxiao.gin@gmail.com
* @version 0.0.1
* @date 2016-06-24 17:05:47
**/

#ifndef AROBOT_WHISPER_H
#define AROBOT_WHISPER_H

#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#define DEFAULT_QUEUE_SIZE 1000
#define UPDATESTRING "JointUpdateCheck"

/** The enum of the JointID 
 * Used for unify the joint ID 
 */
enum JointID {L_THIGH_ROLL, L_THIGH_YAW, L_THIGH_PITCH, L_CALF_PITCH, L_FOOT_PITCH, L_FOOT_YAW, R_THIGH_ROLL, R_THIGH_YAW, R_THIGH_PITCH, R_CALF_PITCH, R_FOOT_PITCH, R_FOOT_YAW, JOINT_NUM};


/** The name array of the JointID 
 * Used for simplify the name strings 
 */

static const char *jointCommandTable[JOINT_NUM] = {
	"/legs11/left_thigh_roll_position_controller/command",
	"/legs11/left_thigh_joint_yaw_position_controller/command",
	"/legs11/left_thigh_joint_pitch_position_controller/command",
	"/legs11/left_calf_pitch_position_controller/command", 
	"/legs11/left_foot_joint_pitch_position_controller/command",
	"/legs11/left_foot_joint_yaw_position_controller/command",
	"/legs11/right_thigh_roll_position_controller/command",
	"/legs11/right_thigh_joint_yaw_position_controller/command",
	"/legs11/right_thigh_joint_pitch_position_controller/command",
	"/legs11/right_calf_joint_pitch_position_controller/command",
	"/legs11/right_foot_joint_pitch_position_controller/command",
	"/legs11/right_foot_joint_yaw_position_controller/command"
};

static const char *jointNameTable[JOINT_NUM] = {
	"l_leg_mhx",
	"l_leg_uhz", 
	"l_leg_lhy",
	"l_leg_kny",
	"l_leg_uay",
	"l_leg_lax",
	"r_leg_mhx",
	"r_leg_uhz",
	"r_leg_lhy",
	"r_leg_kny",
	"r_leg_uay",
	"r_leg_lax"
};

/* The relationship between the two table is in xx.yaml */
/* TODO : It means we can parse the file and set this table automictially */

/**
 * The abstract class encapulates the controller for adapting the commands to real scene.
 */
class WhisperVirtualBase 
{
	public:
		/**
		 * Initialize the publishers.
		 * @param[in] nh the NodeHandle for NodeHandle::advertise
		 * @return The status of the operator 
		 */
		virtual bool InitPublishers(ros::NodeHandle &nh) = 0;

		/**
		 * Shutdown the publisher
		 */
		virtual void ShutdownPublishers(void) = 0;

		/**
		 * Set the size of the queue 
		 * @param[in] The size
		 */
		virtual void SetQueueSize(uint32_t size) = 0;
		/**
		 * Get the queue 
		 * @return The size
		 */
		virtual uint32_t GetQueueSize(void) = 0;

		/**
		 * Set and Pub the position command to the Joint
		 * @param[in] joint The joint enum ID. ex. L_THIGHT_ROLL
		 * @param[in] position The value of the position (rad)
		 * @return The status of the operation 
		 */

		virtual bool PubOnePosition(enum JointID joint, double positions) = 0;

		/**
		 * Set the position command to the Joint
		 * @param[in] joint The joint enum ID. ex. L_THIGHT_ROLL
		 * @param[in] position The value of the position (rad)
		 * @return The status of the operation 
		 */
		virtual bool SetOnePosition(enum JointID joint, double positions) = 0;
		
		/**
		 * Set the position with jointState structure
		 * @param[in] js the joint states 
		 */
		virtual bool SetPositions(const sensor_msgs::JointState & js) = 0;

		/**
		 * Send the position commands to the Joint.
		 * @Warning : The length of array must the JOINT_NUM.
		 * @param[in] positions The value array of the positions (rad)
		 * @return The status of the operation 
		 */
		virtual bool PubPositions(void) = 0;

};

/**
 * The class encapulates the controller for adapting the commands to Gazebo and HardWare.
 */
class WhisperGazebo : public WhisperVirtualBase
{
	public:
		/**
		 * The constructor function
		*/
		WhisperGazebo(uint32_t queueSize = DEFAULT_QUEUE_SIZE)
		{
			this->queueSize = queueSize;
			checkFlag = true;
		}
		/**
		 * The destructor function
		*/
		~WhisperGazebo()
		{
			this->ShutdownPublishers();
		}

		/**
		 * Initialize the publishers.
		 * @param[in] nh the NodeHandle for NodeHandle::advertise
		 * @return The status of the operator 
		 */
		bool InitPublishers(ros::NodeHandle &nh);

		/**
		 * Shutdown the publisher
		 */
		void ShutdownPublishers(void);

		/**
		 * Set the size of the queue 
		 * @param[in] The size
		 */
		void SetQueueSize(uint32_t size);
		/**
		 * Get the queue 
		 * @return The size
		 */
		uint32_t GetQueueSize(void);

		/**
		 * Set and Pub the position command to the Joint
		 * @param[in] joint The joint enum ID. ex. L_THIGHT_ROLL
		 * @param[in] position The value of the position (rad)
		 * @return The status of the operation 
		 */

		bool PubOnePosition(enum JointID joint, double positions);

		/**
		 * Set the position command to the Joint
		 * @param[in] joint The joint enum ID. ex. L_THIGHT_ROLL
		 * @param[in] position The value of the position (rad)
		 * @return The status of the operation 
		 */

		bool SetOnePosition(enum JointID joint, double positions);

		/**
		 * Set the position with jointState structure
		 * @param[in] js the joint states 
		 */
		bool SetPositions(const sensor_msgs::JointState & js);

		/**
		 * Send the position commands to the Joint.
		 * @Warning : The length of array must the JOINT_NUM.
		 * @param[in] positions The value array of the positions (rad)
		 * @return The status of the operation 
		 */
		bool PubPositions(void);

		/** 
		 * Get the name of the joint (URDF)
		 * @param[in] joint The joint enum ID. ex. L_THIGHT_ROLL
		 * @return The string of the path/name
		 */
		const char* GetThePathOfJoint(enum JointID joint);

	private:
		uint32_t queueSize; /**< The queue size of the topic */
		ros::Publisher pubArray[JOINT_NUM]; /**< The publisher array */
		double pubValue[JOINT_NUM]; /**< The value array */

		ros::Publisher updateCheck; /**< The check flag to synchronize the position publishers */
		bool checkFlag;

		// Funcs
		bool PubCheckFlag(void);
};


/**
 * The class encapulates the controller for adapting the commands to Rviz.
 */
class WhisperRZ : public WhisperVirtualBase
{
	public:
		/**
		 * The constructor function
		*/
		WhisperRZ(uint32_t queueSize = DEFAULT_QUEUE_SIZE)
		{
			this->queueSize = queueSize;

			// setup the joint jointState
			this->jointState.header.stamp = ros::Time::now();

			// resize the joint state

			this->jointState.name.resize(JOINT_NUM);
			this->jointState.position.resize(JOINT_NUM);

			for(int i = 0; i != JOINT_NUM; ++i)
			{
				this->jointState.name[i] = jointNameTable[i];
				this->jointState.position[i] = 0; // set the origin position 0 
			}
		}
		/**
		 * The constructor function
		*/
		~WhisperRZ()
		{
			this->ShutdownPublishers(); // shutdown the publisher.
		}
		/**
		 * Initialize the publishers.
		 * @param[in] nh the NodeHandle for NodeHandle::advertise
		 * @return The status of the operator 
		 */
		bool InitPublishers(ros::NodeHandle &nh);

		/**
		 * Shutdown the publisher
		 */
		void ShutdownPublishers(void);

		/**
		 * Set the size of the queue 
		 * @param[in] The size
		 */
		void SetQueueSize(uint32_t size);
		/**
		 * Get the queue 
		 * @return The size
		 */
		uint32_t GetQueueSize(void);

		/**
		 * Set and Pub the position command to the Joint
		 * @param[in] joint The joint enum ID. ex. L_THIGHT_ROLL
		 * @param[in] position The value of the position (rad)
		 * @return The status of the operation 
		 */

		bool PubOnePosition(enum JointID joint, double positions);

		/**
		 * Set the position command to the Joint
		 * @param[in] joint The joint enum ID. ex. L_THIGHT_ROLL
		 * @param[in] position The value of the position (rad)
		 * @return The status of the operation 
		 */

		bool SetOnePosition(enum JointID joint, double positions);

		/**
		 * Set the position with jointState structure
		 * @param[in] js the joint states 
		 */
		bool SetPositions(const sensor_msgs::JointState & js);

		/**
		 * Send the position commands to the Joint.
		 * @Warning : The length of array must the JOINT_NUM.
		 * @param[in] positions The value array of the positions (rad)
		 * @return The status of the operation 
		 */
		bool PubPositions(void);

		/** 
		 * Get the name of the joint (URDF)
		 * @param[in] joint The joint enum ID. ex. L_THIGHT_ROLL
		 * @return The string of the path/name
		 */
		const char* GetThePathOfJoint(enum JointID joint);

	private:
		uint32_t queueSize; /**< The queue size of the topic */
		sensor_msgs::JointState jointState; /**< The joint state */
		ros::Publisher pub; /**< The publisher */
};


class JointInfo
{
	public:
		JointInfo(enum JointID ID)
		{
			this->ID = ID;
		}

		const char * name;
		const char * gazeCommandName;
		enum JointID ID;		
};

/**
 * The class is a manager of different controller while the robot operating.
 */
class WhisperManager
{
	public:
		/**
		 * The constructor function
		*/
		WhisperManager(uint32_t queueSize = DEFAULT_QUEUE_SIZE)
		{
			this->queueSize = queueSize;
			this->RZHandle = new WhisperRZ(queueSize);
			this->gazeboHandle = new WhisperGazebo(queueSize);
		}

		/**
		 * The discontructor function
		 */
		~WhisperManager()
		{
			delete this->RZHandle;
			delete this->gazeboHandle;
		}

		/**
		 * Initialize the publishers.
		 * @param[in] nh the NodeHandle for NodeHandle::advertise
		 * @return The status of the operator 
		 */
		bool InitPublishers(ros::NodeHandle &nh);

		/**
		 * Shutdown the publisher
		 */
		bool ShutdownPublishers(void);

		/**
		 * Set the size of the queue 
		 * @param[in] The size
		 */
		void SetQueueSize(uint32_t size);
		/**
		 * Get the queue 
		 * @return The size
		 */
		uint32_t GetQueueSize(void);

		/** 
		 * Get the info of the joint
		 * @param[in] joint The joint enum ID. ex. L_THIGHT_ROLL
		 * @return The JointInfo of the joint
		 */
		JointInfo GetTheInfoOfJoint(enum JointID joint);

		/**
		 * Set and Pub the position command to the Joint
		 * @param[in] joint The joint enum ID. ex. L_THIGHT_ROLL
		 * @param[in] position The value of the position (rad)
		 * @return The status of the operation 
		 */

		bool PubOnePosition(enum JointID joint, double positions);

		/**
		 * Set the position command to the Joint
		 * @param[in] joint The joint enum ID. ex. L_THIGHT_ROLL
		 * @param[in] position The value of the position (rad)
		 * @return The status of the operation 
		 */

		bool SetOnePosition(enum JointID joint, double positions);

		/**
		 * Set the position with jointState structure
		 * @param[in] js the joint states 
		 */
		bool SetPositions(const sensor_msgs::JointState & js);

		/**
		 * Send the position commands to the Joint.
		 * @Warning : The length of array must the JOINT_NUM.
		 * @param[in] positions The value array of the positions (rad)
		 * @return The status of the operation 
		 */
		bool PubPositions(void);

	private:
		uint32_t queueSize; /**< The queue size of the topic */
		class WhisperRZ* RZHandle; /**< The pointor of HardWare controller */
		class WhisperGazebo* gazeboHandle; /**< The pointor of Gazebo controller */

};
#endif

