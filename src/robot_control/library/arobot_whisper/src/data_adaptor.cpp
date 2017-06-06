/**
* @file data_adaptor.cpp
* @detail the cpp of the data_adaptor
* @author icePie
* @email sunxiao.gin@gmail.com
* @version 0.0.1
* @date 2016-08-10 16:35:03
**/

#include <std_msgs/Float64.h>
#include <data_adaptor.h>

#include <iostream>

using namespace arobotDB;
/* DataAdaptor Gazebo */
bool DataAdaptorGazebo::dataSend(const DataSend &dataSend)
{
	// Publish the joint jointState
	// TODO check the data is ready or not here
	this->gazeboHandle.SetPositions(dataSend.jointState);
	this->gazeboHandle.PubPositions();
	return true;
}

bool DataAdaptorGazebo::dataReceive(DataReceive &dataReceive)
{
	// TODO
}

/* DataAdaptor Rviz */
bool DataAdaptorRviz::dataSend(const DataSend &dataSend)
{
	// Publish the joint jointState
	// TODO check the data is ready or not here
	this->rvizHandle.SetPositions(dataSend.jointState);
	this->rvizHandle.PubPositions();
	return true;
}

bool DataAdaptorRviz::dataReceive(DataReceive &dataReceive)
{
	// TODO
}

