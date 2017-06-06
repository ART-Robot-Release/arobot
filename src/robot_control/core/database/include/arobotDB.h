/**
* @file arobot.h
* @detail The header file of the robot database
* @author Galaxy2416
* @email sunxiao.gin@gmail.com
* @version 0.0.1
* @date 2016-08-02 15:05:39
**/

#ifndef AROBOTDB_H
#define AROBOTDB_H

// C++
#include <string>
#include <chrono>
#include <deque>
#include <iostream>
#include <mutex>

// Boost For Jason
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
// Ros
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>

#define sensor_ arobot_

#include <sensor_msgs/JointState.h>
// #include <arobot_msg/JointState.h>

#define DEFAULT_DATA_LENGTH 100
// differ the re and send in futu#define 
namespace arobotDB
{
	/**
	 * The class contains all the recieve datas 
	 */
	class SixForceSensor
	{
		public:
			SixForceSensor(float xf=0, float yf=0, float zf=0, float xt=0, float yt=0, float zt=0)
			{
				xForce = xf;
				xTorque = xt;
				yForce = yf;
				yTorque = yt;
				zForce = zf;
				zTorque = zt;
				state = 0;
			}

			void calWithZero(const SixForceSensor & ft)
			{
				xForce -= ft.xForce;
				yForce -= ft.yForce;
				zForce -= ft.zForce;

				xTorque -= ft.xTorque;
				yTorque -= ft.yTorque;
				zTorque -= ft.zTorque;
			}

			boost::property_tree::ptree getThePTree(void);

			float xForce;
			float xTorque;
			float yForce;
			float yTorque;
			float zForce;
			float zTorque;
			uint32_t state;

		private:
			boost::property_tree::ptree root;
	};

	class NineAxis
	{
		public:
			NineAxis(float p=0, float r=0, float y=0, float t=0)
			{
				pitch = p;
				roll = r;
				yaw = y;
				gx = 0;
				gy = 0;
				gz = 0;
				temp = t;
				state = 0;
			}

			float pitch;
			float roll;
			float yaw;
			float gx;
			float gy;
			float gz;
			float temp;
			uint32_t state;

			boost::property_tree::ptree getThePTree(void);

		private:
			boost::property_tree::ptree root;
	};

	class DataReceive
	{
		public:
			DataReceive()
			{
				this->jointStateReady = false;
				// TODO: init jointState
			}
			// interfaces here.
			inline bool checkJointState(void) { return this->jointStateReady; }
			inline bool setJointStateReady(void) { this->jointStateReady = true; }
			inline bool clearJointStateReady(void) { this->jointStateReady = false; }

			inline sensor_msgs::JointState getJointState(void) { return this->jointState; }
			inline void setJointState(const sensor_msgs::JointState & js) { this->jointState = js; }
			inline void setTime(const ros::Time &t) { this->time = t; }
			inline ros::Time getTime(void) { return this->time; }
			

			boost::property_tree::ptree getThePTree(void);

			// TODO: set up data function. 

			/*
			   [sensor_msgs/JointState]:
			   std_msgs/Header header
			   uint32 seq
			   time stamp
			   string frame_id
			   string[] name
			   float64[] position
			   float64[] velocity
			   float64[] effort
			   */

			sensor_msgs::JointState jointState; /**< The joint state */
			// TODO：add new sensor data here.
			struct 
			{
				SixForceSensor sixForceLeft;
				SixForceSensor sixForceRight;
				NineAxis posture;
			} sensorData;

			ros::Time time; 

		private:
			bool jointStateReady;
			boost::property_tree::ptree root;
	};

	/**
	 * The class contains all the send datas 
	 */
	class DataSend
	{
		public:
			DataSend()
			{
				this->jointStateReady = false;
				// TODO: init jointState
			}
			// interfaces here.
			inline bool checkJointState(void) { return this->jointStateReady; }
			inline bool setJointStateReady(void) { this->jointStateReady = true; }
			inline bool clearJointStateReady(void) { this->jointStateReady = false; }

			inline sensor_msgs::JointState getJointState(void) { return this->jointState; }
			inline void setJointState(const sensor_msgs::JointState & js) { this->jointState = js; }

			inline void setTime(const ros::Time &t) { this->time = t; }
			inline ros::Time getTime(void) { return this->time; }


			boost::property_tree::ptree getThePTree(void);

			sensor_msgs::JointState jointState; /**< The joint state */
			ros::Time time;
		
		private:
			bool jointStateReady;
			boost::property_tree::ptree root;
	};


	/**
	 * The frame of one loop.
	 */
	class DataFrame
	{
		public:
			DataFrame(unsigned int id=0)
			{
				frameID = id;
			}

			// interfaces here.
			// inline 	std::chrono::steady_clock::time_point getReceiveTimePoint(void) { return this->receiveTp; }
			// inline 	std::chrono::steady_clock::time_point getSendTimePoint(void) { return this->sendTp; }
			// inline 	std::chrono::steady_clock::time_point setReceiveTimePoint(const std::chrono::steady_clock::time_point & tp) { this->receiveTp = tp; return this->receiveTp; }
			// inline 	std::chrono::steady_clock::time_point setSendTimePoint(const std::chrono::steady_clock::time_point & tp) { this->sendTp = tp; return this->sendTp; }

			
			inline DataReceive getReceiveData(void) { return this->receiveData; }
			inline DataSend getSendData(void) { return this->sendData; }
			inline void setFrameID(unsigned int id) { this->frameID = id;}
			inline unsigned int getFrameID(void) { return this->frameID ;}
			

			bool checkReceiveDataReady(void);
			bool checkSendDataReady(void);
			ros::Duration calTheDuration(void);
			
			/* public data */
			DataReceive receiveData ; /**< The received data */
			DataSend sendData ; /**< The sent data */
			ros::Duration interval; /**< The send time - receive time */


			boost::property_tree::ptree getThePTree(void);


		private:
			unsigned int frameID; /**< The ID of the frame */
			boost::property_tree::ptree root;
			// std::chrono::steady_clock::time_point receiveTp; /**< The time point of receiving the data. */
			// std::chrono::steady_clock::time_point sendTp; /**< The time point of sending the data. */
	};

	/**
	 * The data dataQueue.
	 */
	class DataQueue
	{
		public:
			// APIs

			static DataQueue* getInstance();

			inline DataFrame& getTheLastFrame(void)
			{
				return this->dataQueue.back();
			}

			inline DataSend& getTheLastSendData(void)
			{
				return this->dataQueue.back().sendData;
			}

			inline DataReceive& getTheLasReceiveData(void)
			{
				return this->dataQueue.back().receiveData;
			}

			void setup(int reserveSize) 
			{
				this->reserveSize = reserveSize;
			}

			int pushDataFrame(DataFrame);
			DataFrame& createDataFrame(void);

			int reserveSize;
			std::deque<DataFrame> dataQueue;
			

		protected:
			// protect those functions
			DataQueue()
			{
				reserveSize = DEFAULT_DATA_LENGTH;
				dataQueue = std::deque<DataFrame>();
			}
			~DataQueue()
			{
				if((_instance != nullptr))
						delete _instance;
			}
			DataQueue(const DataQueue&) = delete;
			DataQueue& operator=(const DataQueue&) = delete;
		private:
			static DataQueue* _instance; // singleton mode
			static std::mutex _m;

	};
	
	// pushThe
}
#endif // end AROBOTDB_H

