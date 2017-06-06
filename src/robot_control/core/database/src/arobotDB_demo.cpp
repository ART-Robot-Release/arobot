#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <iostream>

#include <arobotDB.h>

#define SYSTEM_RATE 10
#define DATA_LENGTH 20
#define JOINT_NUM 26

using namespace arobotDB;
using namespace std;

void printDatas(DataQueue * d)
{
	ROS_INFO_STREAM("The reserved size is "
			<<d->reserveSize<<endl
			<<"The size is "<<d->dataQueue.size()<<endl
			<<"Data:");
	for(int i=0; i != d->dataQueue.size(); ++i)
	{
		if( i == 0)
			ROS_INFO_STREAM("The send data "<<i<<".[0] is: "
					<<d->dataQueue[i].sendData.jointState.position[0]<<endl
					<<"The ready state is: "
					<<d->dataQueue[i].sendData.checkJointState());
		else
			ROS_INFO_STREAM("The send data "<<i<<".[0] is: "
					<<d->dataQueue[i].sendData.jointState.position[0]<<endl
					<<"The ready state is: "
					<<d->dataQueue[i].sendData.checkJointState()<<endl
					<<"The interval is: "<< d->dataQueue[i].interval.toSec());

				//	<<"The time is: "<<(chrono::duration_cast<chrono::duration<double>>)(d.dataQueue[i].getSendTimePoint() - d.dataQueue[i-1].getSendTimePoint()).count());
	}
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "arobotDB_demo_node");
	ros::NodeHandle nh("~");

	// test jason
	
	SixForceSensor ft(1,2,3,4,5,6);
	NineAxis imu(11,22,33,44);
	
	boost::property_tree::ptree pt1 = ft.getThePTree();
	boost::property_tree::ptree pt2 = imu.getThePTree();

	boost::property_tree::write_json(std::cout,pt1);
	boost::property_tree::write_json(std::cout,pt2);

	DataQueue *datas = DataQueue::getInstance();
	DataSend send_data;
	// The length of the data
	datas->setup(DATA_LENGTH);
	//The system rate
	ros::Rate rate(SYSTEM_RATE);
	
	sensor_msgs::JointState	js ;
	js.name.resize(JOINT_NUM);
	for(int i=0; i!=js.name.size(); ++i)
		js.name[i] = std::to_string(i);
	js.position.resize(JOINT_NUM);
	js.velocity.resize(JOINT_NUM);
	js.effort.resize(JOINT_NUM);

	while(ros::ok())
	{
		//Create a new data firstly
		DataFrame &frame = datas->createDataFrame();

		/* TODO: Receive The data */
		frame.receiveData.setTime(ros::Time::now());
		/* TODO: Publish The data Okay signal */
		rate.sleep();
		frame.sendData.setTime(ros::Time::now());
		frame.sendData.setJointState(js);

		frame.receiveData.setTime(ros::Time::now());
		frame.receiveData.setJointState(js);
		frame.receiveData.sensorData.sixForceLeft = ft;
		frame.receiveData.sensorData.sixForceRight = ft;
		frame.receiveData.sensorData.posture = imu;

		
		/* Get the sendData from database */
		send_data = datas->getTheLastSendData();

		boost::property_tree::ptree pt = frame.getThePTree();

		boost::property_tree::write_json(std::cout,pt);
		/* TODO: Check and Send The data */


		frame.calTheDuration();
		printDatas(datas);
	}
	return 0;
}
