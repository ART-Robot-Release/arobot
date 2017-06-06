#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <math.h>
#include <string.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <iostream>

#include <arobotDB.h>
#include <data_adaptor.h>


using namespace arobotDB;
using namespace std;

void printDatas( DataQueue & d)
{
	ROS_INFO_STREAM("The reserved size is "
			<<d.reserveSize<<endl
			<<"The size is "<<d.dataQueue.size()<<endl
			<<"Data:");
	for(int i; i != d.dataQueue.size(); ++i)
	{
		if( i == 0)
			ROS_INFO_STREAM("The send data "<<i<<".[0] is: "
					<<d.dataQueue[i].sendData.jointState.position[0]<<endl
					<<"The ready state is: "
					<<d.dataQueue[i].sendData.checkJointState());
		else
			ROS_INFO_STREAM("The send data "<<i<<".[0] is: "
					<<d.dataQueue[i].sendData.jointState.position[0]<<endl
					<<"The ready state is: "
					<<d.dataQueue[i].sendData.checkJointState()<<endl
					<<"The interval is: "<< d.dataQueue[i].interval.toSec());

				//	<<"The time is: "<<(chrono::duration_cast<chrono::duration<double>>)(d.dataQueue[i].getSendTimePoint() - d.dataQueue[i-1].getSendTimePoint()).count());
	}
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "control_demo_db");
	ros::NodeHandle nh("~");

	
	// create the database.
	DataQueue *datas = DataQueue::getInstance();
	// The length of the data
	datas->setup(20);
	ros::Rate rate(10);
	/*
	//
	sensor_msgs::JointState	js ;
	js.name.resize(JOINT_NUM);
	js.position.resize(JOINT_NUM);
	//seed
	srand((unsigned)time(NULL));

	// The rviz adaptor
	DataAdaptor* rvizAda = DataAdaptorFactory::createAdaptor(RVIZ, nh); 

	// data
	double angle = 0;
	double i = 0.1;
	while(ros::ok())
	{
		if(angle >= 0.5)
			i = -0.1;
		if(angle <= -0.5)
			i = 0.1;
		angle += i;

		// generate the js data.
		js.position[L_THIGH_PITCH] = angle;
		js.position[R_THIGH_PITCH] = angle;

		// create the data frame
		datas->createDataFrame();

		// set the SendData frame
		DataFrame &frame = datas->getTheLastFrame();

		frame.receiveData.setTime(ros::Time::now());
		
		// frame.setSendTimePoint(chrono::steady_clock::now());
		frame.sendData.setJointState(js) ;
		frame.sendData.setJointStateReady();

		// Send the data 
		rvizAda->dataSend(frame.sendData);

		frame.sendData.setTime(ros::Time::now());
		frame.calTheDuration();

		printDatas(*datas);
		cout<<endl<<endl<<endl;
		rate.sleep();
	}
	*/
	return 0;
}
