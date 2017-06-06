/**
* @file arobotDB.cpp
* @detail The main database of arobot.
* @author icePie
* @email sunxiao.gin@gmail.com
* @version 0.0.1
* @date 2016-08-02 07:02:04
**/

#include <iostream>
#include "arobotDB.h"

namespace arobotDB
{
	/*
	std::string getTheJason(boost::property_tree::ptree& root)
	{
		std::stringstream stream;
		root.write_json(stream, root);
		
		std::string str;
		stream << str;
		return str;
	}
	*/

	boost::property_tree::ptree getThePTree(sensor_msgs::JointState js)
	{
		boost::property_tree::ptree root;
		for(int i=0; i != js.name.size(); ++i)
		{
			root.put<double>(js.name[i]+".position", js.position[i]);
			root.put<double>(js.name[i]+".velocity", js.velocity[i]);
			root.put<double>(js.name[i]+".effort", js.effort[i]);
		}
		return root;
	}


	boost::property_tree::ptree SixForceSensor::getThePTree(void)
	{
		root.put<float>("fx", this->xForce);
		root.put<float>("fy", this->yForce);
		root.put<float>("fz", this->zForce);
		root.put<float>("tx", this->xTorque);
		root.put<float>("ty", this->yTorque);
		root.put<float>("tz", this->zTorque);
		return root;
	}

	boost::property_tree::ptree NineAxis::getThePTree(void)
	{
		root.put<float>("pitch", this->pitch);
		root.put<float>("roll", this->roll);
		root.put<float>("yaw", this->yaw);
		root.put<float>("temp", this->temp);
		return root;
	}

	boost::property_tree::ptree DataReceive::getThePTree(void)
	{
		
		root.put<double>("time", this->time.toSec());
		root.put_child("sensorData.sixForceLeft", this->sensorData.sixForceLeft.getThePTree());
		root.put_child("sensorData.sixForceRight", this->sensorData.sixForceRight.getThePTree());
		root.put_child("sensorData.posture", this->sensorData.posture.getThePTree());
		root.put_child("jointState",arobotDB::getThePTree(this->jointState));
		return root;
	}

	boost::property_tree::ptree DataSend::getThePTree(void)
	{
		root.put<double>("time", this->time.toSec());
		/*
		root.put_child("sensorData.sixForceLeft", this->sendData.sixForceLeft.getThePTree());
		root.put_child("sensorData.sixForceRight", this->sendData.sixForceRight.getThePTree());
		root.put_child("sensorData.posture", this->sendData.posture.getThePTree());
		*/
		root.put_child("jointState",arobotDB::getThePTree(this->jointState));
		return root;
	}

	boost::property_tree::ptree DataFrame::getThePTree(void)
	{
		root.put<double>("interval", this->calTheDuration().toSec());
		root.put("id", this->getFrameID());
		root.put_child("receiveData",this->receiveData.getThePTree());
		root.put_child("sendData",this->sendData.getThePTree());
		return root;
	}

	/*
	std::string SixForceSensor::getTheJason(void)
	{
		boost::property_tree::ptree roo_l = getThePTree();
		std::stringstream stream;
		root.write_json(stream, root_l);
		
		std::string str;
		stream << str;
		return str;
	}
	*/



	/*

	std::string NineAxis::getTheJason(void)
	{
		boost::property_tree::ptree roo_l = getThePTree();
		std::stringstream stream;
		root.write_json(stream, root_l);
		
		std::string str;
		stream << str;
		return str;
	}

	*/

	/* Initialization DataQueue*/

	ros::Duration DataFrame::calTheDuration(void)
	{
		return this->sendData.time - this->receiveData.time;
	}

	
	int DataQueue::pushDataFrame(DataFrame data)
	{
		assert(this->reserveSize >= dataQueue.size()); 
		if(this->dataQueue.size() < this->reserveSize)
			this->dataQueue.push_back(data);
		else 
		{
			this->dataQueue.pop_front();
			this->dataQueue.push_back(data);
		}
		return this->dataQueue.size();
	}

	 DataFrame& DataQueue::createDataFrame(void)
	{
		DataFrame data;
	 	this->pushDataFrame(data);
		return this->getTheLastFrame();
	}
	 

	 std::mutex DataQueue::_m;
	 DataQueue* DataQueue::_instance = nullptr;

	 DataQueue* DataQueue::getInstance()
	 {
		 // check lock
		 if (_instance == nullptr)
		 {
			 std::lock_guard<std::mutex> guard(_m);
				 if(_instance == nullptr) {
				 	_instance = new DataQueue();
				 }
		 }
		 return _instance;
	 }

}

