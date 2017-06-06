/**
* @file data_ataptor.h
* @detail the adaptor database -> gazebo/RVIZ/Hardware
* @author icePie
* @email sunxiao.gin@gmail.com
* @version 0.0.1
* @date 2016-08-10 15:42:37
**/
#ifndef DATA_ADAPTOR_H
#define DATA_ADAPTOR_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

// Local
#include <arobotDB.h>
#include "arobot_whisper.h"

enum ATAPTOR_TYPE { GAZEBO, RVIZ, ATAPTOR_TYPE_NUM };
/**
 * The abstract class encapulates the adaptor 
 */

using namespace arobotDB;

class DataAdaptor
{
	public:
		/**
		 * Send the data out
		 * @param[in] dataSend the data would be sent out.
		 */
		virtual bool dataSend(const DataSend &dataSend) = 0;

		/**
		 * Receive the data
		 * @param[out] dataReceive the instance to receive the data.
		 */
		virtual bool dataReceive(DataReceive &dataReceive) = 0;
};

/**
 * The class encapulates the controller for adapting the commands to Gazebo.
 */
class DataAdaptorGazebo : public DataAdaptor
{
	public:
		/**
		 * The constructor function
		 * @param[in] nh the NodeHandle for NodeHandle::advertise
		*/
		DataAdaptorGazebo(ros::NodeHandle &nh, uint32_t queueSize = DEFAULT_QUEUE_SIZE):
		gazeboHandle(queueSize)
		{
			gazeboHandle.InitPublishers(nh); // init the publish.
		}
		/**
		 * Send the data out to gazebo.
		 * @param[in] dataSend the data would be sent to gazebo.
		 */
		bool dataSend(const DataSend &dataSend);
		/**
		 * Send the data out to gazebo.
		 * @param[in] dataReceive the from gazebo.
		 */
		bool dataReceive(DataReceive &dataReceive);

	private:
		WhisperGazebo gazeboHandle; /**< The  gazebo control handle */
};

/**
 * The class encapulates the controller for adapting the commands to RVIZ.
 */
class DataAdaptorRviz : public DataAdaptor
{
	public:
		/**
		 * The constructor function
		 * @param[in] nh the NodeHandle for NodeHandle::advertise
		*/
		DataAdaptorRviz(ros::NodeHandle &nh, uint32_t queueSize = DEFAULT_QUEUE_SIZE):
		rvizHandle(queueSize)
		{
			this->rvizHandle.InitPublishers(nh); // init the publish.
		}
		/**
		 * Send the data out to gazebo.
		 * @param[in] dataSend the data would be sent to gazebo.
		 */
		bool dataSend(const DataSend &dataSend);
		/**
		 * Send the data out to gazebo.
		 * @param[in] dataReceive the from gazebo.
		 */
		bool dataReceive(DataReceive &dataReceive);

	private:
		WhisperRZ rvizHandle; /**< The  rviz ( some hardware interface ) control handle */
};

/**
 * The simple factory of the data adaptor
 */
class DataAdaptorFactory
{
	public :
		static DataAdaptor* createAdaptor(enum ATAPTOR_TYPE type, ros::NodeHandle &nh, uint32_t queueSize = DEFAULT_QUEUE_SIZE)
		{
			DataAdaptor *temp = NULL;
			switch(type)
			{
				case GAZEBO:
					temp = new DataAdaptorGazebo(nh, queueSize);
					break;
				case RVIZ:
					temp = new DataAdaptorRviz(nh, queueSize);
					break;
				default:
					break;
			}
			return temp;
		}
};

#endif // DATA_ADAPTOR_H
