
// ROS
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <ros/console.h>

// system
#include <boost/date_time.hpp>
#include <boost/bind.hpp>
#include <thread>
#include <math.h>
#include <string.h>
#include <iostream>
#include <signal.h>


#include <arobot_hw.h>
// Msg
#include <arobot_msgs/ForceSix.h>
#include <arobot_msgs/ImuBasic.h>

#define PUB_RATE 100.0
#define SENSOR_PUB_RATE 10.0

#include <fstream>

// external database

#include <arobotDB.h>
#include <extDB.h>
#include "../../database/include/arobotDB.h"

// enum JointID {L_THIGH_ROLL, L_THIGH_YAW, L_THIGH_PITCH, L_CALF_PITCH, L_FOOT_PITCH, L_FOOT_YAW, R_THIGHT_ROLL, R_THIGH_YAW, R_THIGH_PITCH, R_CALF_PITCH, R_FOOT_PITCH, R_FOOT_YAW, JOINT_NUM};

// WARNING : Every controller load process needs one cm->update.
// WARNING : Every controller load process needs one cm->update.
// WARNING : Every controller load process needs one cm->update.

static volatile int signum = 0;

void sigme(int signo)

{
      if(signo == SIGINT)
	  {
        printf("Receive SIGINT.\n");
		signum = 1;
	  }
}

void echo_joints_states(controller_manager::ARobotHW *robot)
{
	//	system("clear");
		for(unsigned int i = 0; i != robot->n_dof_; i++) {
 //           if(robot->joint_effort_[i] >= 1)
			ROS_INFO_STREAM_NAMED("controller main demo ", std::setprecision(5)<<std::setw(8)<<"Joint : "<<i<< " - "<< robot->joint_names_[i]<<" \t{P: "<< robot->joint_position_[i]
                <<"\t\t V: "<<robot->joint_velocity_[i]<<"\t\t C: "<<robot->joint_effort_[i]<<"\t}");
		}

			ROS_INFO_STREAM_NAMED("controller main demo ", std::setprecision(5)<<std::setw(8)<<"Six Force : " << robot->l_foot_ft.zForce);
}

void echo_csv_joints_states(std::ostream &f, controller_manager::ARobotHW *robot)
{
	//	system("clear");
		for(unsigned int i = 0; i != robot->n_dof_; i++) {
			f<<robot->data_send.jointState.position[i]<<',';
			f<<robot->data_receive.jointState.position[i];
			f<<",";
		}
		for(unsigned int i = 0; i != robot->n_dof_; i++) {
			f<<robot->data_send.jointState.effort[i]<<',';
			f<<robot->data_receive.jointState.effort[i];
			f<<",";
		}
		for(unsigned int i = 0; i != robot->n_dof_; i++) {
			f<<robot->data_send.jointState.velocity[i]<<',';
			f<<robot->data_receive.jointState.velocity[i];
			if(i == robot->n_dof_ - 1)
				f<<std::endl;
			else
				f<<",";
		}
}

void echo_cvs_joints_states_title(std::ostream &f, controller_manager::ARobotHW *robot)
{
	//	system("clear");
		for(unsigned int i = 0; i != robot->n_dof_; i++) {
			f<<robot->joint_names_[i]<<"+p_send"<<",";
			f<<robot->joint_names_[i]<<"+p_receive";
			f<<",";
		}
		for(unsigned int i = 0; i != robot->n_dof_; i++) {
			f<<robot->joint_names_[i]<<"+c_send"<<",";
			f<<robot->joint_names_[i]<<"+c_receive";
			f<<",";
		}
		for(unsigned int i = 0; i != robot->n_dof_; i++) {
			f<<robot->joint_names_[i]<<"+s_send"<<",";
			f<<robot->joint_names_[i]<<"+s_receive";
			if(i == robot->n_dof_ - 1)
				f<<std::endl;
			else
				f<<",";
		}
}


arobot_msgs::ForceSix get_ft_msg(const arobotDB::SixForceSensor &data)
{
	arobot_msgs::ForceSix msg;

	msg.force[0] = data.xForce;
	msg.force[1] = data.yForce;
	msg.force[2] = data.zForce;
	msg.torque[0] = data.xTorque;
	msg.torque[1] = data.yTorque;
	msg.torque[2] = data.zTorque;
	msg.state = data.state;

    msg.header.stamp = ros::Time::now();

	return msg;
}

arobot_msgs::ImuBasic get_nine_axis_msg(const arobotDB::NineAxis &data)
{
	arobot_msgs::ImuBasic msg;
	msg.pitch = data.pitch;
	msg.roll = data.roll;
	msg.yaw = data.yaw;
	msg.temperature = data.temp;
	msg.gyro[0] = data.gx;
	msg.gyro[1] = data.gy;
	msg.gyro[2] = data.gz;

    msg.header.stamp = ros::Time::now();

	return msg;
}

// global for debugging
ros::Publisher l_ft_pub ;
ros::Publisher r_ft_pub ;

ros::Publisher nine_axis_pub ;

std::string filename;


void update(controller_manager::ControllerManager* cm, controller_manager::ARobotHW* robot)
{
	static unsigned int id = 0;

	using namespace arobotDB;

	ros::Rate rate(PUB_RATE);

    ROS_INFO_STREAM_NAMED("MAIN", "cm load");
    ros::Duration(2).sleep();
	for(unsigned int i = 0; i != robot->n_dof_; i++) {
		robot->read(ros::Time::now(), ros::Duration(1.0/PUB_RATE));
		cm->update(ros::Time::now(), ros::Duration(1.0/PUB_RATE), false);
    }
    ros::Duration(2).sleep();
    ROS_INFO_STREAM_NAMED("MAIN", "cm load finish");


	// data files
	std::ofstream f(filename);
	if (!f.is_open()) {
		ROS_ERROR("please change the default path where contains data.txt, can not open it %s", filename.c_str());
		exit(0);
	}
	// echo_cvs_joints_states_title(f, robot);

	// external database
	// signal(SIGINT, sigme);

	DataQueue *datas = DataQueue::getInstance();
	datas->setup(50);

	ExtDatabase *db =  ExtDatabaseFactory::createDB("redis");
	if(!db->connect("127.0.0.1", 6379))
    	ROS_DEBUG_STREAM_NAMED("MAIN", "redis connection failed.");

	while(ros::ok())
	{
	//	ROS_INFO_STREAM_NAMED("controler main demo", ros::Time::now());

		robot->read(ros::Time::now(), ros::Duration(1.0/PUB_RATE));

		// if(robot.GetUpdateFlag()) // Synchronize all the joint commands.

		cm->update(ros::Time::now(), ros::Duration(1.0/PUB_RATE), false);

		robot->write(ros::Time::now(), ros::Duration(1.0/PUB_RATE));

		// echo_joints_states(robot);
		// echo_csv_joints_states(f, robot);

		// restore in data base
		DataFrame &frame = datas->createDataFrame();
		frame.sendData = robot->data_send;
		frame.receiveData = robot->data_receive;
		frame.setFrameID(id++);
		if(!db->insertDataFrame("hi", frame))
    		ROS_DEBUG_STREAM_NAMED("MAIN", "redis insert failed.");


		rate.sleep();
	}

	db->save();

	f.close();
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "controller_main");
	ros::NodeHandle  nh = ros::NodeHandle ("/arobot");


	if(argc == 2)
		filename = argv[1];
	else {
		filename = ros::package::getPath("control_demo")+"/src/testcases/data.cvs";
	}

	ROS_INFO_STREAM_NAMED("controller_main_demo", filename);


	//if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
	//	ros::console::notifyLoggerLevelsChanged();
	//}

	l_ft_pub = nh.advertise<arobot_msgs::ForceSix>("l_foot_ft", 1000);
	r_ft_pub = nh.advertise<arobot_msgs::ForceSix>("r_foot_ft", 1000);

	nine_axis_pub = nh.advertise<arobot_msgs::ImuBasic>("body_nine_axis", 1000);

	controller_manager::ARobotHW robot;

	robot.Load();



    ROS_INFO_STREAM_NAMED("controller_main_demo", "Loading controller_manager");
	controller_manager::ControllerManager cm(&robot, nh);

	// ros::Time the_last((ros::Time::now()));
    //ros::Duration(3).sleep();

	auto pub_sensors = [&robot]()
	{

		ros::Rate rate(SENSOR_PUB_RATE);

		while(ros::ok())
		{
			l_ft_pub.publish(get_ft_msg(robot.l_foot_ft));
			r_ft_pub.publish(get_ft_msg(robot.r_foot_ft));

			nine_axis_pub.publish(get_nine_axis_msg(robot.body_nine_axis));
			
		}

		rate.sleep();
	};

	std::thread t(boost::bind(update, &cm, &robot)); // DO NEED A THREAD !!! FOR REAL-TIME SAFE. SEE CONTROLLER_BASE CLASS
	std::thread ts(pub_sensors);

	ros::spin();

    t.join();
	ts.join();

	ROS_INFO_STREAM_NAMED("controller main demo ", "main process finish.");

	return 0;
}
