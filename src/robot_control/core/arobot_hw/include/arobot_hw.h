#ifndef A_ROBOT_HW_H
#define A_ROBOT_HW_H

// C++
#include <unordered_map>
#include <map>
#include <vector>
#include <deque>
// Boost
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

// ROS
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <std_msgs/Bool.h>

// URDF

#include <urdf/model.h>

// datas types
#include <arobotDB.h>

// ros_control
#include <controller_manager/controller_manager.h>
#include <control_toolbox/pid.h>
#include <transmission_interface/transmission_parser.h>
#include <transmission_interface/transmission_info.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>

// message
#include <arobot_hw/SelfDetection.h>
#include <arobot_hw/DriverDetection.h>
#include <arobot_hw/ActuatorDetection.h>
#include <arobot_hw/SensorDetection.h>
#include <arobot_hw/ActuatorControl.h>
#include <arobot_hw/SensorCalibration.h>

#ifdef ECAT
#include "ecat_init.h"
#endif

#define ROBOT_PARAM_NAME "robot_description"

#define UPDATESTRING "JointUpdateCheck"

namespace controller_manager
{

    struct  i2tLimitInfo
	{
		struct valStep
		{
			valStep(double dur = 0, double cur = 0, double e = 0): duration(dur), val(cur), eng(e)
			{}
			double duration;
			double val;
			double eng;
		};

		i2tLimitInfo():window(0), nominal_current(0), limit(0), sum_duration(0), sum_energy(0), status(true)
		{
		}

		bool i2tCheck(const std::string &name, double duration, double current)
		{

			assert(name == this->name);
			if(window == 0 || limit == 0)
				return false;
			current = fabs(current);
			double eng_step = current > nominal_current ? (current * current - nominal_current * nominal_current) * duration : 0;

			sum_energy += eng_step;
			sum_duration += duration;

			valStep val_step(duration, current, eng_step);
			valque.push_back(val_step);

			while(sum_duration > window)
			{
				if(valque.empty())
					break;

				sum_duration -= valque.front().duration;
				sum_energy -= eng_step;
				valque.pop_front();
			}
			return (sum_energy < limit);
		}

		std::string name;
		std::deque <valStep> valque;
		double window;
		double nominal_current;
		double limit;
		double sum_duration;
		double sum_energy;

		bool status;
	};
	struct JointLimitInfo
	{
		JointLimitInfo()
		{
			has_limits = false;
			has_soft_limits = false;
		}

		void extraLimitInfo(double &lower, double &upper, double &vel, double &effort)
		{
			lower = -std::numeric_limits<double>::max();
			upper = std::numeric_limits<double>::max();
			effort = std::numeric_limits<double>::max();
			vel = std::numeric_limits<double>::max();

			if(!has_limits)
				return;

			if (limits.has_position_limits)
			{
				lower = limits.min_position;
				upper = limits.max_position;
			}
			if (limits.has_effort_limits)
				effort = limits.max_effort;
			if(limits.has_velocity_limits)
				vel = limits.max_velocity;
		}

		joint_limits_interface::JointLimits limits;
		joint_limits_interface::SoftJointLimits soft_limits;
		bool has_limits;
		bool has_soft_limits;
	};

    struct CurrentBits
    {
        int value:16; // value
        bool C:1; // Current Loop : 1 enable
        bool S:1; // Speed Loop : 1 enable
        bool P:1; // Position Loop : 1 enable
        bool O:1; // Power On 1
        bool T:1; // Command Control 1 enable
        int reserve:10;
        bool ok:1;
    };

    union EtherCatReg
    {
        uint32_t reg;
		float sensor;
        CurrentBits bits;
    };


	class ARobotHW : public hardware_interface::RobotHW
	{
		public:
			ARobotHW();
			~ARobotHW()
            {
#ifdef ECAT
		    artrobot::ecat::EcatAdmin::shutdown();
#endif
            }


			void read(const ros::Time &time, const ros::Duration &period);
			void write(const ros::Time &time, const ros::Duration &period);
			bool Load();
			bool Init( const std::string& robot_namespace,
					ros::NodeHandle model_nh,
					const urdf::Model *const urdf_model,
					std::vector<transmission_interface::TransmissionInfo> transmissions);

			bool GetUpdateFlag (void);  // The function is unused now.

			ros::Time GetTime(void);

			ros::Duration GetPeriod(void);

			std::string GetURDF(std::string param_name) const;

			bool ParseTransmissionsFromURDF(const std::string& urdf_string);

			//	protected:

			// Methods used to control a joint.
			enum ControlMethod {EFFORT, POSITION, POSITION_PID, VELOCITY, VELOCITY_PID, COMBINE};

			// Register the limits of the joint specified by joint_name and joint_handle. The limits are
			// retrieved from joint_limit_nh. If urdf_model is not NULL, limits are retrieved from it also.
			// Return the joint's type, lower position limit, upper position limit, and effort limit.
			void RegisterJointLimits(const std::string& joint_name,
					const hardware_interface::JointHandle& joint_handle,
					const ControlMethod ctrl_method,
					const ros::NodeHandle& joint_limit_nh,
					const urdf::Model *const urdf_model,
					int *const joint_type, double *const lower_limit,
					double *const upper_limit, double *const effort_limit);

			void RegisterJointLimits(const std::string& joint_name,
					const hardware_interface::JointHandle& joint_handle,
					const ControlMethod ctrl_method,
					const JointLimitInfo& limit_info);

			void GetJointLimits(const std::string& joint_name,
					const ros::NodeHandle& joint_limit_nh,
					const urdf::Model *const urdf_model,
					JointLimitInfo &limits,
					int *const joint_type);


			// joints
			unsigned int n_dof_;

			hardware_interface::JointStateInterface    js_interface_;
			hardware_interface::EffortJointInterface   ej_interface_;
			hardware_interface::PositionJointInterface pj_interface_;
			hardware_interface::VelocityJointInterface vj_interface_;

			joint_limits_interface::EffortJointSaturationInterface   ej_sat_interface_;
			joint_limits_interface::EffortJointSoftLimitsInterface   ej_limits_interface_;
			joint_limits_interface::PositionJointSaturationInterface pj_sat_interface_;
			joint_limits_interface::PositionJointSoftLimitsInterface pj_limits_interface_;
			joint_limits_interface::VelocityJointSaturationInterface vj_sat_interface_;
			joint_limits_interface::VelocityJointSoftLimitsInterface vj_limits_interface_;

			std::vector<std::string> joint_names_;
			std::vector<int> joint_types_;

			std::vector<JointLimitInfo> joint_limits_;

			std::vector<double> joint_lower_limits_;
			std::vector<double> joint_upper_limits_;
			std::vector<double> joint_effort_limits_;
			std::vector<double> joint_velocity_limits_;

			std::vector<ControlMethod> joint_control_methods_;
			std::vector<control_toolbox::Pid> pid_controllers_; // Usually not be used for hardware

			std::vector<double> joint_position_;
			std::vector<double> joint_velocity_;
			std::vector<double> joint_current_;
			std::vector<double> joint_effort_;
			std::vector<double> joint_effort_coefficent_;

			std::vector<i2tLimitInfo> joint_i2t_limit_;

			std::vector<double> joint_effort_command_;
			std::vector<double> joint_position_command_;
			std::vector<double> last_joint_position_command_;
			std::vector<double> joint_velocity_command_;

			// sensors

			arobotDB::SixForceSensor zero_l_foot_ft;
			arobotDB::SixForceSensor zero_r_foot_ft;

			arobotDB::SixForceSensor l_foot_ft;
			arobotDB::SixForceSensor r_foot_ft;
			arobotDB::NineAxis body_nine_axis;

			// datas
			arobotDB::DataSend data_send;
			arobotDB::DataReceive data_receive;

			// e_stop_active_ is true if the emergency stop is active.
            //
			bool e_stop_active_, last_e_stop_active_;

			int getRos2Cal(const std::string & str)
			{
				return getName2Cal(str, this->name_map_ros);
			}
			int getRobot2Cal(const std::string & str)
			{
				return getName2Cal(str, this->name_map_robot);
			}

		private:

			// transmissions
			std::vector<transmission_interface::TransmissionInfo> transmissions_;

			// Node Handles
			ros::NodeHandle model_nh_; // namespaces to robot name

			// Strings
			std::string robot_namespace_;
			std::string robot_description_;

			// robot model
			urdf::Model robot_model_;

			ros::Time time;
			ros::Duration period;

			ros::Subscriber updateCheckSub;
			bool checkFlag; // should be setted by subcribing the info from Joints Publishers.

			// JOINTS MAP
			bool initJointsMap(void);
			std::map<std::string, int> name_map_ros;
			std::map<std::string, int> name_map_robot;

			int getName2Cal(const std::string & str, std::map<std::string, int> & map);
            //ECAT
#ifdef ECAT
            std::unordered_map<int, std::shared_ptr<AngleToNumber>> angle_cal_dict_;
            std::unordered_map<int, std::string> motor_to_joint_name_;
#endif
			// Functions
			void setUpdateFlag (const std_msgs::Bool value); //callbacks

			bool initEffortInfo(void);

			// get the ethercat id from name
			std::unordered_map<std::string, uint32_t> name_to_id_dir;

			bool initEcatID(void);
			uint32_t getEcatID(const std::string & str);
            void initEcatInterface(void);

            //adc 2 current

            double adc2current(int value);


			/** \name ROS Service API
			 *\{*/
			bool selfDetectionSrv(arobot_hw::SelfDetection::Request &req,
					arobot_hw::SelfDetection::Response &resp);  /**< the controller and commucation */
			bool driverDetectionSrv(arobot_hw::DriverDetection::Request &req,
					arobot_hw::DriverDetection::Response &resp);
			bool actuatorDetectionSrv(arobot_hw::ActuatorDetection::Request &req,
					arobot_hw::ActuatorDetection::Response &resp);
			bool sensorDetectionSrv(arobot_hw::SensorDetection::Request &req,
					arobot_hw::SensorDetection::Response &resp);

			enum class actuatorStates {RUN, SHUTDOWN,POWER, CalZERO} actSta_;

			bool actuatorControlSrv(arobot_hw::ActuatorControl::Request &req,
								arobot_hw::ActuatorControl::Response &resp);

			bool sensorCalibrationSrv(arobot_hw::SensorCalibration::Request &req,
								arobot_hw::SensorCalibration::Response &resp);


		mutable boost::mutex services_lock_;
		ros::ServiceServer srv_self_detect_, srv_driver_detect_, srv_actuator_detect_, srv_sensor_detect_, srv_actuator_control_, srv_sensor_calibration_;
		/*\}*/
	};
}


#endif


