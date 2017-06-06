#include <boost/date_time.hpp>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <math.h>
#include <string.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <iostream>

#include <arobot_whisper.h>


// enum JointID {L_THIGH_ROLL, L_THIGH_YAW, L_THIGH_PITCH, L_CALF_PITCH, L_FOOT_PITCH, L_FOOT_YAW, R_THIGHT_ROLL, R_THIGH_YAW, R_THIGH_PITCH, R_CALF_PITCH, R_FOOT_PITCH, R_FOOT_YAW, JOINT_NUM};

int main(int argc, char **argv) {

	ros::init(argc, argv, "control_demo");
	ros::NodeHandle nh("~");
	WhisperManager control(1000); // Set the control manage
	control.InitPublishers(nh);
	// Try to move the 
	ros::Rate rate(10);
	double angle = 0; 
	double i = 0.1;
	while(ros::ok())
	{
		if(angle >= 0.5)
			i = -0.1;
		if(angle <= -0.5)
			i = 0.1;
		angle += i;

//			control.PubOnePosition(L_THIGH_PITCH, (double)angle);
//			control.PubOnePosition(R_THIGH_PITCH, (double)angle);
			control.SetOnePosition(L_THIGH_PITCH, (double)angle);
			control.SetOnePosition(R_THIGH_PITCH, (double)angle);
			control.PubPositions();
		rate.sleep();
	}
	control.ShutdownPublishers();
	return 0;
}
