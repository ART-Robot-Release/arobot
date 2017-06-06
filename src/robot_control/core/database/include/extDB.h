/**
* @file extDB.h
* @detail The header file of the external robot database
e @author Galaxy2416
* @email sunxiao.gin@gmail.com
* @version 0.0.1
* @date 2017-03-29 18:28:39
**/

#ifndef EXTDB_H
#define EXTDB_H

// Local

#include <arobotDB.h>

// C++
#include <string>
#include <chrono>
#include <deque>
#include <iostream>
#include <mutex>

// Ros
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>

// redis
#include "hiredis.h"  

using namespace arobotDB;

class ExtDatabase
{
	public:

		/**
		 * Connect the database 
		 */
		virtual bool connect(std::string ip="127.0.0.1", int port = 6379) = 0;

		/**
		 * Disconnect the database 
		 */
		virtual bool disconnect() = 0;

		/**
		 * insert the data frame
		 * @param[in] d dataframe
		 */
		virtual bool insertDataFrame(std::string name, DataFrame& d) = 0;

		virtual bool save(std::string name = "dump.rdb") = 0;
};

/**
 * The class encapulates the controller for adapting the commands to Gazebo.
 */
class RedisDB : public ExtDatabase
{

	// TODO
	public:
		/**
		 * The constructor function
		 * @param[in] nh the NodeHandle for NodeHandle::advertise
		*/
		RedisDB()
		{
			redisCon = nullptr;
		}

		~RedisDB()
		{
 			disconnect();
		}
		
		bool connect(std::string ip="127.0.0.1", int port = 6379) override
		{
			redisCon = redisConnect(ip.c_str(), port);  
			if(redisCon != nullptr && redisCon->err)  
			{  
				printf("connection error: %s\n",redisCon->errstr);
				redisCon = nullptr;
				return false;  
			}
			else if(redisCon == nullptr)
				return false;

			redisReply *reply = (redisReply*)redisCommand(redisCon, "CONFIG SET appendonly yes");  
			freeReplyObject(reply);  
			reply = (redisReply*)redisCommand(redisCon, "CONFIG SET appendfsync everysec");  
			freeReplyObject(reply);  
			reply = (redisReply*)redisCommand(redisCon, "CONFIG SET dbfilename dump.rdb");   // TODO : change to time;
			freeReplyObject(reply);  
			reply = (redisReply*)redisCommand(redisCon, "CONFIG SET dir ./");   // TODO : change to time;
			freeReplyObject(reply);  

			return true;
		}

		bool disconnect(void) override
		{
			if(redisCon != nullptr) 
				redisFree(redisCon); 
			return true;
		}

		bool insertDataFrame(std::string name, DataFrame & d) override;

		bool save(std::string name = "dump.rdb") override;

	private:
			redisContext *redisCon;
};


/**
 * The simple factory of the data adaptor
 */
class ExtDatabaseFactory
{
	public :
		static ExtDatabase*  createDB(std::string db_name = "redis")
		{
			ExtDatabase *temp = nullptr;
			if(db_name == "redis" || db_name == "Redis")
					temp = new RedisDB();
			return temp;
		}
};



#endif // EXTDB_H
