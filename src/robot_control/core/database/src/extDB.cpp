// Local

#include <arobotDB.h>
#include "extDB.h"

// C++
#include <string>
#include <iostream>
#include <mutex>

// Ros
#include <ros/ros.h>

// redis
#include "hiredis.h"  


bool RedisDB::insertDataFrame(std::string name, DataFrame & d)
{
    if(redisCon == nullptr)
        return false;

	std::stringstream stream;

	boost::property_tree::ptree pt = d.getThePTree();
	boost::property_tree::write_json(stream, pt);

	redisReply *reply = (redisReply*)redisCommand(redisCon, "RPUSH %s %s", name.c_str(), stream.str().c_str());  

	// printf("RPUSH %s %s", name.c_str(), stream.str().c_str());
	// TODO: DO SOMETHING WITH reply
	// if (nullptr != reply) {
	//	printf("%s %d", reply->str, reply->len);
	freeReplyObject(reply);  

	return true;
}


bool RedisDB::save(std::string name)
{
    if(redisCon == nullptr)
		return false;
	redisReply *reply = (redisReply*)redisCommand(redisCon, "CONFIG SET dbfilename %s", name.c_str());   // TODO : change to time;
	freeReplyObject(reply);  
	reply = (redisReply*)redisCommand(redisCon, "BGSAVE");  
	printf("%s %d", reply->str, reply->len);
	freeReplyObject(reply); 
}
