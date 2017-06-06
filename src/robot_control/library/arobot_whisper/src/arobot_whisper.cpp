/**
* @file control_manager.cpp
* @detail The cpp file of control
* @author Galaxy2416
rint "All tests passed."
* @email sunxiao.gin@gmail.com
* @version 0.0.1
* @date 2016-06-24 17:05:21
**/

#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <arobot_whisper.h>

#include <iostream>

/* The functions in WhisperRZ */

bool WhisperRZ::InitPublishers(ros::NodeHandle &nh)
{
	this->pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 1); // the name of the
	return this->pub ? true : false;
}

void WhisperRZ::ShutdownPublishers(void)
{
	this->pub.shutdown();
}

bool WhisperRZ::PubOnePosition(enum JointID joint, double position)
{
	std::cout<<" Send the commend to "<<jointNameTable[joint]<<" with "<<position<<std::endl;
	if (SetOnePosition(joint, position))
		return PubPositions();
	else
		return false;
}

bool WhisperRZ::SetOnePosition(enum JointID joint, double position)
{
	this->jointState.position[joint] = position;
	return true;
}

bool WhisperRZ::SetPositions(const sensor_msgs::JointState & js)
{
	assert(js.position.size() == (int) JOINT_NUM);
	
	for(int i = 0; i != JOINT_NUM; ++i)
	{
		this->jointState.position[i] = js.position[i];
	}
	return true;
}

bool WhisperRZ::PubPositions(void)
{
	this->jointState.header.stamp = ros::Time::now();
	this->pub.publish(this->jointState);
	return true;
}


void WhisperRZ::SetQueueSize(uint32_t queueSize)
{
	this->queueSize = queueSize;
}

uint32_t WhisperRZ::GetQueueSize(void)
{
	return this->queueSize;
}

const char* WhisperRZ::GetThePathOfJoint(enum JointID joint)
{
	return jointNameTable[joint];
}

/* The functions in WhisperGazebo */

bool WhisperGazebo::InitPublishers(ros::NodeHandle &nh)
{
	for(int i = 0; i != JOINT_NUM; ++i)
		{
			if(this->pubArray[i] = nh.advertise<std_msgs::Float64>(jointCommandTable[i], this->queueSize))
				continue;			
			else
				return false;
		}
	if(this->updateCheck = nh.advertise<std_msgs::Bool>(UPDATESTRING, this->queueSize))
		return true;
	else
		return false;
}



void WhisperGazebo::ShutdownPublishers(void)
{
	for(int i = 0; i != JOINT_NUM; ++i)
		{
			this->pubArray[i].shutdown();
		}
	this->updateCheck.shutdown();
}

bool WhisperGazebo::PubOnePosition(enum JointID joint, double position)
{
	std::cout<<" Send the command to "<<jointCommandTable[joint]<<" with "<<position<<std::endl;

	std_msgs::Float64 value ;
	value.data = position;
	pubArray[joint].publish(value);
	
	SetOnePosition(joint, position); // save the new position.
	return true;
}

bool WhisperGazebo::SetOnePosition(enum JointID joint,  double position)
{
	this->pubValue[joint] = position;
	return true;
}

bool WhisperGazebo::SetPositions(const sensor_msgs::JointState & js)
{
	assert(js.position.size() == (int) JOINT_NUM);
	for(int i = 0; i != JOINT_NUM; ++i)
	{
		if(!SetOnePosition((enum JointID)i, js.position[i]))
			return false;
	}
	return true;
}

// pub this checkFlag 
bool WhisperGazebo::PubCheckFlag(void) 
{
	std_msgs::Bool value;
	value.data = this->checkFlag;
	this->updateCheck.publish(value);
	return value.data;
}

bool WhisperGazebo::PubPositions(void)
{
	checkFlag = false;
	PubCheckFlag();

	for(int i = 0; i != JOINT_NUM; ++i)
	{
		if(!PubOnePosition((enum JointID)i, this->pubValue[i]))
			return false;
	}

	checkFlag = true;
	PubCheckFlag();
	return true;
}

void WhisperGazebo::SetQueueSize(uint32_t queueSize)
{
	this->queueSize = queueSize;
}

uint32_t WhisperGazebo::GetQueueSize(void)
{
	return this->queueSize;
}

const char* WhisperGazebo::GetThePathOfJoint(enum JointID joint)
{
	return jointCommandTable[joint];
}

/* The function of this manager */

// TODO : Whisper the output by the param later */

bool WhisperManager::InitPublishers(ros::NodeHandle &nh)
{
	bool ret = true;
	ret &= this->RZHandle->InitPublishers(nh);
	ret &= this->gazeboHandle->InitPublishers(nh);
	return ret;
}

bool WhisperManager::ShutdownPublishers(void)
{
	this->RZHandle->ShutdownPublishers();
	this->gazeboHandle->ShutdownPublishers();
	return true;
}




JointInfo WhisperManager::GetTheInfoOfJoint(enum JointID joint)
{
	JointInfo info(joint);
	info.name = jointNameTable[joint];
	info.gazeCommandName = jointCommandTable[joint];

	return info;
}


bool WhisperManager::PubOnePosition(enum JointID joint, double position)
{
	if(!this->RZHandle->PubOnePosition(joint, position))
		return false;
	if(!this->gazeboHandle->PubOnePosition(joint, position))
		return false;
	return true;	
}

bool WhisperManager::SetOnePosition(enum JointID joint,  double position)
{
	if(!this->RZHandle->SetOnePosition(joint, position))
		return false;
	if(!this->gazeboHandle->SetOnePosition(joint, position))
		return false;
	return true;
}

bool WhisperManager::SetPositions(const sensor_msgs::JointState & js)
{
	if(!this->RZHandle->SetPositions(js))
		return false;
	if(!this->gazeboHandle->SetPositions(js))
		return false;
	return true;
}

bool WhisperManager::PubPositions(void)
{
	if(!this->RZHandle->PubPositions())
		return false;
	if(!this->gazeboHandle->PubPositions())
		return false;
	return true;
}

void WhisperManager::SetQueueSize(uint32_t queueSize)
{
	this->queueSize = queueSize;
	this->RZHandle->SetQueueSize(queueSize);
	this->gazeboHandle->SetQueueSize(queueSize);
}


uint32_t WhisperManager::GetQueueSize(void)
{
	return this->queueSize;
}

