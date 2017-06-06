/**
 * @file arobot_hw.c
 * @detail
 * @author IcePie
 * @email sunxiao@artrobot.com
 * @version 0.0.1
 * @date 2016-11-02 15:41:09
 **/


// Boost
#include <boost/bind.hpp>

// Ros
#include <angles/angles.h>

// Local
#include "arobot_hw.h"
#include "../include/arobot_hw.h"

namespace controller_manager
{
	namespace
	{
		float reg2Sensor(uint32_t value)
		{
			EtherCatReg conv;
			conv.reg = value;
			return conv.sensor;
		}

		double clamp(const double val, const double min_val, const double max_val)
		{
			return std::min(std::max(val, min_val), max_val);
		}

		bool update_l_ft_zero = true;
		bool update_r_ft_zero = true;

	}

	ARobotHW::ARobotHW()
	{
		using namespace hardware_interface;

		// Get the info from urdf
		/*
		// Initialize raw data
		joint_position_.resize(JOINT_NUM);
		joint_velocity_.resize(JOINT_NUM);
		joint_effort_.resize(JOINT_NUM);
		joint_effort_command_.resize(JOINT_NUM);
		joint_velocity_command_.resize(JOINT_NUM);
		joint_name_.resize(JOINT_NUM);

		for(int i = 0; i != JOINT_NUM; ++i)
		{
		// TODO: The initial position should be parsed from the config files.

		joint_name_[i] = jointNameTable[i];
		joint_position_[i] = 0.0;
		joint_velocity_[i] = 0.0;
		joint_effort_[i] = 0.1;
		joint_effort_command_[i] = 0.0;
		joint_velocity_command_[i] = 0.0;
		}

		// Populate hardware interfaces
		for(int i = 0; i != JOINT_NUM; ++i)
		{
		js_interface_.registerHandle(JointStateHandle(joint_name_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]));
		ej_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[i]), &joint_effort_command_[i]));
		vj_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[i]), &joint_velocity_command_[i]));
		}

		registerInterface(&js_interface_);
		registerInterface(&ej_interface_);
		registerInterface(&vj_interface_);

*/

		// init the updateCheck
		checkFlag = true;
	}

	void ARobotHW::read(const ros::Time &time, const ros::Duration &period)
	{
#ifdef ECAT
        ROS_DEBUG_NAMED("ecat", "read begin");
        using namespace artrobot::ecat;
		ecrt_master_receive(EcatAdmin::master);
		ecrt_domain_process(EcatAdmin::domain1);
        ROS_DEBUG_NAMED("ecat", "read master ok");
#endif
		// TODO: Read the data from ethercat
		for(unsigned int j=0; j < n_dof_; j++)
		{
			if (joint_types_[j] == urdf::Joint::PRISMATIC)
			{
				// joint_position_[j] = sim_joints_[j]->GetAngle(0).Radian();
				joint_position_[j] = joint_position_command_[j]; //TODO
			}
			else
			{
#ifdef ECAT
        ROS_DEBUG_NAMED("ecat", "read position");
        if(EcatAdmin::motor_dict[getEcatID(joint_names_[j])] == nullptr)
            ROS_INFO_STREAM_NAMED("ecat", "null  " <<joint_names_[j]<<" "<< EcatAdmin::motor_dict[getEcatID(joint_names_[j])]);


       // ROS_INFO_STREAM_NAMED("ecat","number"<<EcatAdmin::motor_dict[j+1]->get_t_pdo_reg_index(0));
		joint_position_[j] = angle_cal_dict_[getEcatID(joint_names_[j])]->inv_calc(
						EC_READ_U32(EcatAdmin::domain1_pd +
							PdoRegTable::regs[EcatAdmin::motor_dict[getEcatID(joint_names_[j])]->get_t_pdo_reg_index(0)]));

		// robot -> ros
		joint_position_[j] = joint_position_[j] *  getName2Cal(joint_names_[j], name_map_ros) * getName2Cal(joint_names_[j], name_map_robot);

	    joint_velocity_[j] = angle_cal_dict_[getEcatID(joint_names_[j])]->inv_calc(
						EC_READ_U32(EcatAdmin::domain1_pd +
							PdoRegTable::regs[EcatAdmin::motor_dict[getEcatID(joint_names_[j])]->get_t_pdo_reg_index(1)]));

        union EtherCatReg current;

        current.reg = (uint32_t) EC_READ_U32(EcatAdmin::domain1_pd +
							PdoRegTable::regs[EcatAdmin::motor_dict[getEcatID(joint_names_[j])]->get_t_pdo_reg_index(2)]);


	    // ROS_INFO_STREAM_NAMED("arobot_hw","' Read the info '" << joint_names_[j]
		//			<<" current " << std::hex <<current.reg);

		joint_current_[j] = adc2current(current.bits.value);

        joint_effort_[j] = joint_effort_coefficent_[j] * joint_current_[j];


        ROS_DEBUG_NAMED("ecat", "read position finish");
#else
				// joint_position_[j] += angles::shortest_angular_distance(joint_position_[j],
				//		sim_joints_[j]->GetAngle(0).Radian());
				joint_position_[j] += angles::shortest_angular_distance(joint_position_[j],joint_position_command_[j]); // TODO
#endif
			}
			//joint_velocity_[j] = sim_joints_[j]->GetVelocity(0);
			// joint_effort_[j] = sim_joints_[j]->GetForce((unsigned int)(0));
            //

			// Debug
			ROS_DEBUG_STREAM_NAMED("arobot_hw","' Read the info '" << joint_names_[j]
					<< "' of position '" << joint_position_[j] << "' of velocity '"<<joint_velocity_[j]
					<<"' of effort '"<< joint_effort_[j]);
		}



#ifdef ECAT
		// get the sensors data
        // artrobot::ecat::EcatAdmin::sync_out_buffer();
		// left
        l_foot_ft.xForce = reg2Sensor(EC_READ_U32(EcatAdmin::domain1_pd + PdoRegTable::regs[EcatAdmin::ft_dict[1]->get_t_pdo_reg_index(0)]));
        l_foot_ft.yForce = reg2Sensor(EC_READ_U32(EcatAdmin::domain1_pd + PdoRegTable::regs[EcatAdmin::ft_dict[1]->get_t_pdo_reg_index(1)]));
        l_foot_ft.zForce = reg2Sensor(EC_READ_U32(EcatAdmin::domain1_pd + PdoRegTable::regs[EcatAdmin::ft_dict[1]->get_t_pdo_reg_index(2)]));
        l_foot_ft.xTorque = reg2Sensor(EC_READ_U32(EcatAdmin::domain1_pd + PdoRegTable::regs[EcatAdmin::ft_dict[1]->get_t_pdo_reg_index(3)]));
        l_foot_ft.yTorque = reg2Sensor(EC_READ_U32(EcatAdmin::domain1_pd + PdoRegTable::regs[EcatAdmin::ft_dict[1]->get_t_pdo_reg_index(4)]));
        l_foot_ft.zTorque = reg2Sensor(EC_READ_U32(EcatAdmin::domain1_pd + PdoRegTable::regs[EcatAdmin::ft_dict[1]->get_t_pdo_reg_index(5)]));
        l_foot_ft.state = EC_READ_U32(EcatAdmin::domain1_pd + PdoRegTable::regs[EcatAdmin::ft_dict[1]->get_t_pdo_reg_index(6)]);

		if(update_l_ft_zero) {
			zero_l_foot_ft = l_foot_ft;
			update_l_ft_zero = false;
		}
		l_foot_ft.calWithZero(zero_l_foot_ft);
		// right
        r_foot_ft.xForce = reg2Sensor(EC_READ_U32(EcatAdmin::domain1_pd + PdoRegTable::regs[EcatAdmin::ft_dict[2]->get_t_pdo_reg_index(0)]));
        r_foot_ft.yForce = reg2Sensor(EC_READ_U32(EcatAdmin::domain1_pd + PdoRegTable::regs[EcatAdmin::ft_dict[2]->get_t_pdo_reg_index(1)]));
        r_foot_ft.zForce = reg2Sensor(EC_READ_U32(EcatAdmin::domain1_pd + PdoRegTable::regs[EcatAdmin::ft_dict[2]->get_t_pdo_reg_index(2)]));
        r_foot_ft.xTorque = reg2Sensor(EC_READ_U32(EcatAdmin::domain1_pd + PdoRegTable::regs[EcatAdmin::ft_dict[2]->get_t_pdo_reg_index(3)]));
        r_foot_ft.yTorque = reg2Sensor(EC_READ_U32(EcatAdmin::domain1_pd + PdoRegTable::regs[EcatAdmin::ft_dict[2]->get_t_pdo_reg_index(4)]));
        r_foot_ft.zTorque = reg2Sensor(EC_READ_U32(EcatAdmin::domain1_pd + PdoRegTable::regs[EcatAdmin::ft_dict[2]->get_t_pdo_reg_index(5)]));
        r_foot_ft.state = EC_READ_U32(EcatAdmin::domain1_pd + PdoRegTable::regs[EcatAdmin::ft_dict[2]->get_t_pdo_reg_index(6)]);


		if(update_r_ft_zero) {
			zero_r_foot_ft = r_foot_ft;
			update_r_ft_zero = false;
		}
		r_foot_ft.calWithZero(zero_r_foot_ft);

		// nine axis
        body_nine_axis.pitch = reg2Sensor(EC_READ_U32(EcatAdmin::domain1_pd + PdoRegTable::regs[EcatAdmin::imu_dict[1]->get_t_pdo_reg_index(0)]));
        body_nine_axis.roll = reg2Sensor(EC_READ_U32(EcatAdmin::domain1_pd + PdoRegTable::regs[EcatAdmin::imu_dict[1]->get_t_pdo_reg_index(1)]));
        body_nine_axis.yaw = reg2Sensor(EC_READ_U32(EcatAdmin::domain1_pd + PdoRegTable::regs[EcatAdmin::imu_dict[1]->get_t_pdo_reg_index(2)]));
		body_nine_axis.gx = reg2Sensor(EC_READ_U32(EcatAdmin::domain1_pd + PdoRegTable::regs[EcatAdmin::imu_dict[1]->get_t_pdo_reg_index(3)]));
		body_nine_axis.gy = reg2Sensor(EC_READ_U32(EcatAdmin::domain1_pd + PdoRegTable::regs[EcatAdmin::imu_dict[1]->get_t_pdo_reg_index(4)]));
		body_nine_axis.gz = reg2Sensor(EC_READ_U32(EcatAdmin::domain1_pd + PdoRegTable::regs[EcatAdmin::imu_dict[1]->get_t_pdo_reg_index(5)]));
        body_nine_axis.temp = reg2Sensor(EC_READ_U32(EcatAdmin::domain1_pd + PdoRegTable::regs[EcatAdmin::imu_dict[1]->get_t_pdo_reg_index(6)]));


		for (auto & item : EcatAdmin::item_list) {
			if (item->have_t_pdo()) {
				for (int i = 0; i != item->get_pdo_number(); ++i) {
					PdoBuffer::push_data_byte_4(item->get_t_pdo_buffer_index(i), EC_READ_U32(
							EcatAdmin::domain1_pd + PdoRegTable::regs[item->get_t_pdo_reg_index(i)]));
				}
			}
		}

		if(pthread_mutex_trylock(&PdoBuffer::buffer_mutex) == 0){
			for(int i = 0; i != PdoBuffer::buffer_size; ++i){
				PdoBuffer::tem_buffer[i] = PdoBuffer::in_buffer[i];
			}
			pthread_mutex_unlock(&PdoBuffer::buffer_mutex);
		}
#endif
		// Put datas to receive data type
		data_receive.time = ros::Time::now();
		data_receive.jointState.position = joint_position_;
		data_receive.jointState.effort = joint_effort_;
		data_receive.jointState.velocity = joint_velocity_;
		data_receive.sensorData.sixForceRight = r_foot_ft;
		data_receive.sensorData.sixForceLeft = l_foot_ft;
		data_receive.sensorData.posture = body_nine_axis;
	}

	void ARobotHW::write(const ros::Time &time, const ros::Duration &period)
	{

		boost::mutex::scoped_lock guard(services_lock_);

        // if(actSta_ == actuatorStates::SHUTDOWN)
		//	return;
		// assert(actSta_ == actuatorStates::RUN);

#ifdef ECAT
        using namespace artrobot::ecat;
#endif
		// If the E-stop is active, joints controlled by position commands will maintain their positions.
		if (e_stop_active_)
		{
			if (!last_e_stop_active_)
			{
				last_joint_position_command_ = joint_position_;
				last_e_stop_active_ = true;
			}
			joint_position_command_ = last_joint_position_command_;
		}
		else
		{
			last_e_stop_active_ = false;
		}

        if(actSta_ == actuatorStates::SHUTDOWN || actSta_ == actuatorStates::POWER)
        {
            joint_position_command_ = joint_position_;
        }
        else if(actSta_ == actuatorStates::POWER)
        {
           joint_position_command_ = joint_position_;
        }

		ej_sat_interface_.enforceLimits(period);
		ej_limits_interface_.enforceLimits(period);
		pj_sat_interface_.enforceLimits(period);
		pj_limits_interface_.enforceLimits(period);
		vj_sat_interface_.enforceLimits(period);
		vj_limits_interface_.enforceLimits(period);


		for(unsigned int j=0; j < n_dof_; j++)
		{
			switch (joint_control_methods_[j])
			{
				case EFFORT:
					{
						const double effort = e_stop_active_ ? 0 : joint_effort_command_[j];
						// TODO: sim_joints_[j]->SetForce(0, effort);
					}
					break;

				case POSITION:
					// TODO set the value : sim_joints_[j]->SetPosition(0, joint_position_command_[j]);
					break;

				case POSITION_PID:
					{
						double error;
						switch (joint_types_[j])
						{
							case urdf::Joint::REVOLUTE:
								angles::shortest_angular_distance_with_limits(joint_position_[j],
										joint_position_command_[j],
										joint_lower_limits_[j],
										joint_upper_limits_[j],
										error);
								break;
							case urdf::Joint::CONTINUOUS:
								error = angles::shortest_angular_distance(joint_position_[j],
										joint_position_command_[j]);
								break;
							default:
								error = joint_position_command_[j] - joint_position_[j];
						}

						const double effort_limit = joint_effort_limits_[j];
						const double effort = clamp(pid_controllers_[j].computeCommand(error, period),
								-effort_limit, effort_limit);
						// TODO: sim_joints_[j]->SetForce(0, effort);
					}
					break;

				case VELOCITY:
					// TODO: sim_joints_[j]->SetParam("vel", 0, e_stop_active_ ? 0 : joint_velocity_command_[j]);
					break;

				case VELOCITY_PID:
					{
						double error;
						if (e_stop_active_)
							error = -joint_velocity_[j];
						else
							error = joint_velocity_command_[j] - joint_velocity_[j];
						const double effort_limit = joint_effort_limits_[j];
						const double effort = clamp(pid_controllers_[j].computeCommand(error, period),
								-effort_limit, effort_limit);
						// TODO: sim_joints_[j]->SetForce(0, effort);
						break;
					}
				case COMBINE:
					{
						ROS_DEBUG_STREAM_NAMED("arobot_hw", "The COMBINE writing...");
						// More fine-tuning here
						// effort
						const double effort = e_stop_active_ ? 0 : joint_effort_command_[j];
						// position
						double position = joint_position_command_[j];
						// const double position = joint_position_[j];
						// velocity
						const double velocity = e_stop_active_ ? 0 : joint_velocity_command_[j];

						// map ros -> robot
						position = position * getName2Cal(joint_names_[j], name_map_ros) * getName2Cal(joint_names_[j], name_map_robot);

#ifdef ECAT
                        //ROS_INFO_STREAM_NAMED("test_position_cmd","joint_name: "<<joint_names_[j]<<"\t"<<position);
                        //ROS_INFO_NAMED("arobot_hw", "name--:%s \t %f", joint_names_[j].c_str(), joint_position_command_[j]);
					if(joint_i2t_limit_[j].i2tCheck(joint_names_[j], period.toSec() ,joint_current_[j]))
					{
						EC_WRITE_U32(EcatAdmin::domain1_pd + PdoRegTable::regs[
											 EcatAdmin::motor_dict[getEcatID(joint_names_[j])]->get_r_pdo_reg_index(0)],
									 angle_cal_dict_[getEcatID(joint_names_[j])]->calc(position));

						if(joint_i2t_limit_[j].status != true)
						{
							ROS_INFO_NAMED("arobot_hw", "i2t of joint : %s is safe now.", joint_names_[j].c_str());
                            joint_i2t_limit_[j].status = true;
						}
					}
					else
					{
						if(joint_i2t_limit_[j].status != false)
						{
							ROS_INFO_NAMED("arobot_hw", "i2t of joint : %s overflow.", joint_names_[j].c_str());
                            joint_i2t_limit_[j].status = false
						}
						joint_position_command_ = joint_position_;
						EC_WRITE_U32(EcatAdmin::domain1_pd + PdoRegTable::regs[
						EcatAdmin::motor_dict[getEcatID(joint_names_[j])]->get_r_pdo_reg_index(2)], 0x170000);
					}
#endif
						break;
					}
			}
		}

#ifdef ECAT
			EC_WRITE_U32(EcatAdmin::domain1_pd + PdoRegTable::regs[EcatAdmin::ft_dict[1]->get_r_pdo_reg_index(6)], 1);
			EC_WRITE_U32(EcatAdmin::domain1_pd + PdoRegTable::regs[EcatAdmin::ft_dict[2]->get_r_pdo_reg_index(6)], 1);
#endif

#ifdef ECAT
		// TODO: Write the data to ehercat.
		ecrt_domain_queue(EcatAdmin::domain1);
		ecrt_master_send(EcatAdmin::master);
#endif
		// Put datas to send data type
		data_send.time = ros::Time::now();
		data_send.jointState.position = joint_position_command_;
		data_send.jointState.velocity = joint_velocity_command_;
		data_send.jointState.effort = joint_effort_command_;


		ROS_DEBUG_STREAM_NAMED("arobot_hw", "The writing finish.");
	}

    void ARobotHW::initEcatInterface(void)
    {
#ifdef ECAT
    //ros::Duration(3).sleep();
		ROS_INFO_STREAM_NAMED("arobot_hw", "arobot hardware ecat init start.");
		artrobot::ecat::art_robot_init(nullptr);
		ROS_INFO_STREAM_NAMED("arobot_hw", "arobot hardware ecat init 1.");
		init_ecat_from_ros(model_nh_, angle_cal_dict_, motor_to_joint_name_);
		ROS_INFO_STREAM_NAMED("arobot_hw", "arobot hardware ecat init 2.");
		artrobot::ecat::EcatAdmin::start_for_ros_control();
		ROS_INFO_STREAM_NAMED("arobot_hw", "arobot hardware ecat init finish.");
        //artrobot::ecat::EcatAdmin::shutdown();
#endif
    }

	bool ARobotHW::Load()
	{
		// Check that ROS has been initialized
		if(!ros::isInitialized())
		{
			ROS_FATAL_STREAM_NAMED("arobot_hw","A ROS node has not been initialized, unable to load plugin.");
			return false;
		}
#ifdef ECAT
        initEcatInterface();
#endif


		robot_namespace_ = "/arobot"; // TODO
		robot_description_ = ROBOT_PARAM_NAME;
		// Get parameters/settings for controllers from ROS param server
		// Read urdf from ros parameter server then
		// setup actuators and mechanism control node.
		// This call will block if ROS is not properly initialized.
		model_nh_ = ros::NodeHandle(robot_namespace_);
		// init the robot model
		const std::string urdf_string  = GetURDF(robot_description_);

		if(!robot_model_.initString(urdf_string))
		{
			ROS_FATAL_NAMED("arobot_hw","Could not find robot description parameter.");
			return false;
		}
		// robot_namespace_ = robot_model_.getName(); // default

		// TODO: get the control period from the urdf or some config file

		// get transmissions_ from urdf
		if (!ParseTransmissionsFromURDF(urdf_string))
		{
			ROS_ERROR_NAMED("arobot_hw", "Error parsing URDF for tranmission interfaces \n");
			return false;
		}

	// init the arobot_hw TODO
		if(!this->Init(robot_namespace_, model_nh_, &robot_model_, transmissions_))
		{
			ROS_FATAL_NAMED("arobot_hw","Could not initialize robot simulation interface");
			return false;
		}


		// init the Subscribers
		this->updateCheckSub = model_nh_.subscribe(UPDATESTRING, 1, &ARobotHW::setUpdateFlag, this);
		// init the services
        // ros::ServiceServer srv_self_detect_, srv_driver_detect_, srv_actuator_detect_, srv_sensor_detect_;
		this->srv_self_detect_ = model_nh_.advertiseService("master_controller_detect", &ARobotHW::selfDetectionSrv, this);
		this->srv_driver_detect_ = model_nh_.advertiseService("drivers_detect", &ARobotHW::driverDetectionSrv, this);
		this->srv_actuator_detect_ = model_nh_.advertiseService("actuators_detect", &ARobotHW::actuatorDetectionSrv, this);
		this->srv_sensor_detect_ = model_nh_.advertiseService("sensors_detect", &ARobotHW::sensorDetectionSrv, this);
        this->srv_actuator_control_ = model_nh_.advertiseService("actuator_control", &ARobotHW::actuatorControlSrv, this);
        this->srv_sensor_calibration_ = model_nh_.advertiseService("sensors_calibration", &ARobotHW::sensorCalibrationSrv, this);

		// get the etercat id
		if(!this->initEcatID())
		{
			ROS_FATAL_NAMED("arobot_hw","Could not initialize robot ethercat id table.");
			return false;
		}

		// get the etercat id
		if(!this->initJointsMap())
		{
			ROS_FATAL_NAMED("arobot_hw","Could not initialize joints map tables.");
			return false;
		}

		if(!this->initEffortInfo())
		{

			ROS_FATAL_NAMED("arobot_hw","Could not initialize robot joint effort coefficient.");
			return false;
		}

		return true;
	}

	bool ARobotHW::Init(
			const std::string& robot_namespace,
			ros::NodeHandle model_nh,
			const urdf::Model *const urdf_model,
			std::vector<transmission_interface::TransmissionInfo> transmissions)
	{

		// getJointLimits() searches joint_limit_nh for joint limit parameters. The format of each
		// parameter's name is "joint_limits/<joint name>". An example is "joint_limits/axle_joint".

		const ros::NodeHandle joint_limit_nh(model_nh);

		n_dof_ = transmissions.size();
		unsigned int n_dof_j = urdf_model->joints_.size();
		if(n_dof_ != n_dof_j)
		{
			ROS_WARN_STREAM_NAMED("arobot_hw","Transmission number : " << n_dof_ << " is not the same with the Joints number : "
					<< n_dof_j);
			ROS_WARN_STREAM_NAMED("arobot_hw", "Please check if a \" base_link_to_body \" joint is added." );
		}
		joint_names_.resize(n_dof_);
		joint_types_.resize(n_dof_);

		joint_limits_.resize(n_dof_);

		joint_lower_limits_.resize(n_dof_);
		joint_upper_limits_.resize(n_dof_);
		joint_effort_limits_.resize(n_dof_);
		joint_velocity_limits_.resize(n_dof_);

		joint_control_methods_.resize(n_dof_);
		pid_controllers_.resize(n_dof_);
		joint_position_.resize(n_dof_);
		joint_velocity_.resize(n_dof_);
		joint_current_.resize(n_dof_);
		joint_effort_.resize(n_dof_);
		joint_effort_coefficent_.resize(n_dof_);
		joint_i2t_limit_.resize(n_dof_);
		joint_effort_command_.resize(n_dof_);
		joint_position_command_.resize(n_dof_);
		last_joint_position_command_.resize(n_dof_);
		joint_velocity_command_.resize(n_dof_);

		// Initialize the values
		for(unsigned int j=0; j < n_dof_; j++)
		{
			// Check that this transmission has one joint
			if(transmissions[j].joints_.size() == 0)
			{
				ROS_WARN_STREAM_NAMED("arobot_hw","Transmission " << transmissions[j].name_
						<< " has no associated joints.");
				continue;
			}
			else if(transmissions[j].joints_.size() > 1)
			{
				ROS_WARN_STREAM_NAMED("arobot_hw","Transmission " << transmissions[j].name_
						<< " has more than one joint.");
				continue;
			}

			std::vector<std::string> joint_interfaces = transmissions[j].joints_[0].hardware_interfaces_;
			if (joint_interfaces.empty() &&
					!(transmissions[j].actuators_.empty()) &&
					!(transmissions[j].actuators_[0].hardware_interfaces_.empty()))
			{
				// TODO: Deprecate HW interface specification in actuators in ROS J
				joint_interfaces = transmissions[j].actuators_[0].hardware_interfaces_;
				ROS_WARN_STREAM_NAMED("arobot_hw", "The <hardware_interface> element of tranmission " <<
						transmissions[j].name_ << " should be nested inside the <joint> element, not <actuator>. " <<
						"The transmission will be properly loaded, but please update " <<
						"your robot model to remain compatible with future versions of the plugin.");
			}

			if (joint_interfaces.empty()) {

				ROS_WARN_STREAM_NAMED("arobot_hw", "Joint " << transmissions[j].joints_[0].name_ <<
						" of transmission " << transmissions[j].name_ << " does not specify any hardware interface. " <<
						"Not adding it to the robot hardware simulation.");
				continue;
			}

			else if (joint_interfaces.size() > 1)
			{
				ROS_WARN_STREAM_NAMED("arobot_hw", "Joint " << transmissions[j].joints_[0].name_ <<
						" of transmission " << transmissions[j].name_ << " specifies multiple hardware interfaces.");
				//continue;
			}

			// Add data from transmission
			joint_names_[j] = transmissions[j].joints_[0].name_;
			// TODO： Confirm the data
			joint_position_[j] = 0.0;
			joint_velocity_[j] = 0.0;
			joint_current_[j] = 0.0;
			joint_effort_[j] = 0.0;  // N/m for continuous joints
			joint_effort_command_[j] = 0.0;
			joint_position_command_[j] = 0.0;
			joint_velocity_command_[j] = 0.0;

			const std::string& hardware_interface = joint_interfaces.front();

			// Debug
			ROS_DEBUG_STREAM_NAMED("arobot_hw","Loading joint '" << joint_names_[j]
					<< "' of type '" << hardware_interface << "'");

			// Create joint state interface for all joints
			js_interface_.registerHandle(hardware_interface::JointStateHandle(
						joint_names_[j], &joint_position_[j], &joint_velocity_[j], &joint_effort_[j]));

			// Decide what kind of command interface this actuator/joint has
			hardware_interface::JointHandle joint_handle;

#ifdef GAZEBO_SIM // simulation

			if(hardware_interface == "EffortJointInterface")
			{
				// Create effort joint interface
				joint_control_methods_[j] = EFFORT;
				joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
						&joint_effort_command_[j]);
				ej_interface_.registerHandle(joint_handle);
			}
			else if(hardware_interface == "PositionJointInterface")
			{
				// Create position joint interface
				joint_control_methods_[j] = POSITION;
				joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
						&joint_position_command_[j]);
				pj_interface_.registerHandle(joint_handle);
			}
			else if(hardware_interface == "VelocityJointInterface")
			{
				// Create velocity joint interface
				joint_control_methods_[j] = VELOCITY;
				joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
						&joint_velocity_command_[j]);
				vj_interface_.registerHandle(joint_handle);
			}
			else
			{
				ROS_FATAL_STREAM_NAMED("arobot_hw","No matching hardware interface found for '"
						<< hardware_interface );
				return false;
			}

			RegisterJointLimits(joint_names_[j], joint_handle, joint_control_methods_[j],
					joint_limit_nh, urdf_model,
					&joint_types_[j], &joint_lower_limits_[j], &joint_upper_limits_[j],
					&joint_effort_limits_[j]);

			if (joint_control_methods_[j] != EFFORT)
			{
				// Initialize the PID controller. If no PID gain values are found,	TODO something

				const ros::NodeHandle nh(model_nh, "/arobot_hw/pid_gains/" +
						joint_names_[j]);
				if (pid_controllers_[j].init(nh, true))
				{
					switch (joint_control_methods_[j])
					{
						case POSITION:
							joint_control_methods_[j] = POSITION_PID;
							break;
						case VELOCITY:
							joint_control_methods_[j] = VELOCITY_PID;
							break;
					}
				}
				else
				{
					ROS_WARN_STREAM_NAMED("arobot_hw","No PID gain is found "
							<< hardware_interface );
					//  return false;
				}
			}
#else // Hardware, it means different controller should be set for gazebo and hardware ( usually in .yaml file )

			joint_control_methods_[j] = COMBINE; // joint effort, position and velocity could be controlled in hardware circumstance

			GetJointLimits(joint_names_[j], joint_limit_nh, urdf_model, joint_limits_[j], &joint_types_[j]); // Get the joint limit and type

			// extra the info to limit datas for some obsoleted usage .
			joint_limits_[j].extraLimitInfo(joint_lower_limits_[j], joint_upper_limits_[j], joint_velocity_limits_[j], joint_effort_limits_[j]);

			// effort
			joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
					&joint_effort_command_[j]);
			ej_interface_.registerHandle(joint_handle);

			RegisterJointLimits(joint_names_[j], joint_handle, EFFORT, joint_limits_[j]);

			// position
			joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
					&joint_position_command_[j]);
			pj_interface_.registerHandle(joint_handle);
			RegisterJointLimits(joint_names_[j], joint_handle, POSITION, joint_limits_[j]);

			// velocity
			joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
					&joint_velocity_command_[j]);
			vj_interface_.registerHandle(joint_handle);

			RegisterJointLimits(joint_names_[j], joint_handle, VELOCITY, joint_limits_[j]);

#endif
		}
		// Register interfaces
		registerInterface(&js_interface_);
		registerInterface(&ej_interface_);
		registerInterface(&pj_interface_);
		registerInterface(&vj_interface_);
		// Initialize the emergency stop code.
		e_stop_active_ = false;
		last_e_stop_active_ = false;

		// Initialize the actuators status
        actSta_ = actuatorStates::RUN;

		// init data_s_r names
		data_receive.jointState.name = joint_names_;
		data_send.jointState.name = joint_names_;


		return true;
	}



	ros::Time ARobotHW::GetTime(void)
	{
		return this->time; //TODO
	}

	ros::Duration ARobotHW::GetPeriod(void)
	{

		return this->period;//TODO
	}

	bool ARobotHW::GetUpdateFlag (void)
	{
		return this->checkFlag;
	}

	std::string ARobotHW::GetURDF(std::string param_name) const
	{
		std::string urdf_string;

		// search and wait for robot_description on param server
		while (urdf_string.empty())
		{
			std::string search_param_name;
			if (model_nh_.searchParam(param_name, search_param_name))
			{
				ROS_INFO_ONCE_NAMED("arobot_hw", "arobot hardware is waiting for model"
						" URDF in parameter [%s] on the ROS param server.", search_param_name.c_str());

				model_nh_.getParam(search_param_name, urdf_string);
			}
			else
			{
				ROS_INFO_ONCE_NAMED("arobot_hw", "arobot hardware is waiting for model"
						" URDF in parameter [%s] on the ROS param server.", robot_description_.c_str());

				model_nh_.getParam(param_name, urdf_string);
			}

			usleep(100000);
		}
		ROS_DEBUG_STREAM_NAMED("arobot_hw", "Recieved urdf from param server, parsing...");

		return urdf_string;
	}


	// Register the limits of the joint specified by joint_name and joint_handle. The limits are
	// retrieved from joint_limit_nh. If urdf_model is not NULL, limits are retrieved from it also.
	// Return the joint's type, lower position limit, upper position limit, and effort limit.
	void ARobotHW::RegisterJointLimits(const std::string& joint_name,
			const hardware_interface::JointHandle& joint_handle,
			const ControlMethod ctrl_method,
			const ros::NodeHandle& joint_limit_nh,
			const urdf::Model *const urdf_model,
			int *const joint_type, double *const lower_limit,
			double *const upper_limit, double *const effort_limit)
	{
		*joint_type = urdf::Joint::UNKNOWN;
		*lower_limit = -std::numeric_limits<double>::max();
		*upper_limit = std::numeric_limits<double>::max();
		*effort_limit = std::numeric_limits<double>::max();

		joint_limits_interface::JointLimits limits;
		bool has_limits = false;
		joint_limits_interface::SoftJointLimits soft_limits;
		bool has_soft_limits = false;

		if (urdf_model != NULL)
		{
			const boost::shared_ptr<const urdf::Joint> urdf_joint = urdf_model->getJoint(joint_name);
			if (urdf_joint != NULL)
			{
				*joint_type = urdf_joint->type;
				// Get limits from the URDF file.
				if (joint_limits_interface::getJointLimits(urdf_joint, limits))
					has_limits = true;
				if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits))
					has_soft_limits = true;
			}
		}
		// Get limits from the parameter server.
		if (joint_limits_interface::getJointLimits(joint_name, joint_limit_nh, limits))
			has_limits = true;

		if (!has_limits)
			return;

		if (*joint_type == urdf::Joint::UNKNOWN)
		{
			// Infer the joint type.

			if (limits.has_position_limits)
			{
				*joint_type = urdf::Joint::REVOLUTE;
			}
			else
			{
				if (limits.angle_wraparound)
					*joint_type = urdf::Joint::CONTINUOUS;
				else
					*joint_type = urdf::Joint::PRISMATIC;
			}
		}

		if (limits.has_position_limits)
		{
			*lower_limit = limits.min_position;
			*upper_limit = limits.max_position;
		}
		if (limits.has_effort_limits)
			*effort_limit = limits.max_effort;

		if (has_soft_limits)
		{
			switch (ctrl_method)
			{
				case EFFORT:
					{
						const joint_limits_interface::EffortJointSoftLimitsHandle
							limits_handle(joint_handle, limits, soft_limits);
						ej_limits_interface_.registerHandle(limits_handle);
					}
					break;
				case POSITION:
					{
						const joint_limits_interface::PositionJointSoftLimitsHandle
							limits_handle(joint_handle, limits, soft_limits);
						pj_limits_interface_.registerHandle(limits_handle);
					}
					break;
				case VELOCITY:
					{
						const joint_limits_interface::VelocityJointSoftLimitsHandle
							limits_handle(joint_handle, limits, soft_limits);
						vj_limits_interface_.registerHandle(limits_handle);
					}
					break;
			}
		}
		else
		{
			switch (ctrl_method)
			{
				case EFFORT:
					{
						const joint_limits_interface::EffortJointSaturationHandle
							sat_handle(joint_handle, limits);
						ej_sat_interface_.registerHandle(sat_handle);
					}
					break;
				case POSITION:
					{
						const joint_limits_interface::PositionJointSaturationHandle
							sat_handle(joint_handle, limits);
						pj_sat_interface_.registerHandle(sat_handle);
					}
					break;
				case VELOCITY:
					{
						const joint_limits_interface::VelocityJointSaturationHandle
							sat_handle(joint_handle, limits);
						vj_sat_interface_.registerHandle(sat_handle);
					}
					break;
			}
		}
	}

	// Register the limits of the joint specified by joint_name and joint_handle.
	// The limits are stored in the JointLimitInfo.
	void ARobotHW::RegisterJointLimits(const std::string& joint_name,
			const hardware_interface::JointHandle& joint_handle,
			const ControlMethod ctrl_method,
			const JointLimitInfo& limit_info)
	{
		if (limit_info.has_soft_limits)
		{
			switch (ctrl_method)
			{
				case EFFORT:
					{
						const joint_limits_interface::EffortJointSoftLimitsHandle
							limits_handle(joint_handle, limit_info.limits, limit_info.soft_limits);
						ej_limits_interface_.registerHandle(limits_handle);
					}
					break;
				case POSITION:
					{
						const joint_limits_interface::PositionJointSoftLimitsHandle
							limits_handle(joint_handle, limit_info.limits, limit_info.soft_limits);
						pj_limits_interface_.registerHandle(limits_handle);
					}
					break;
				case VELOCITY:
					{
						const joint_limits_interface::VelocityJointSoftLimitsHandle
							limits_handle(joint_handle, limit_info.limits, limit_info.soft_limits);
						vj_limits_interface_.registerHandle(limits_handle);
					}
					break;
			}
		}
		else
		{
			switch (ctrl_method)
			{
				case EFFORT:
					{
						const joint_limits_interface::EffortJointSaturationHandle
							sat_handle(joint_handle, limit_info.limits);
						ej_sat_interface_.registerHandle(sat_handle);
					}
					break;
				case POSITION:
					{
						const joint_limits_interface::PositionJointSaturationHandle
							sat_handle(joint_handle, limit_info.limits);
						pj_sat_interface_.registerHandle(sat_handle);
					}
					break;
				case VELOCITY:
					{
						const joint_limits_interface::VelocityJointSaturationHandle
							sat_handle(joint_handle, limit_info.limits);
						vj_sat_interface_.registerHandle(sat_handle);
					}
					break;
			}
		}
	}

	// Get the limits of the joint specified by joint_name . The limits are
	// retrieved from joint_limit_nh. If urdf_model is not NULL, limits are retrieved from it also.
	// Return the joint's type, limit, soft_limits.
	void ARobotHW::GetJointLimits(const std::string& joint_name,
			const ros::NodeHandle& joint_limit_nh,
			const urdf::Model *const urdf_model,
			JointLimitInfo &limits,
			int *const joint_type)
	{
		*joint_type = urdf::Joint::UNKNOWN;

		limits.has_limits = false;
		limits.has_soft_limits = false;

		if (urdf_model != NULL)
		{
			const boost::shared_ptr<const urdf::Joint> urdf_joint = urdf_model->getJoint(joint_name);
			if (urdf_joint != NULL)
			{
				*joint_type = urdf_joint->type;
				// Get limits from the URDF file.
				if (joint_limits_interface::getJointLimits(urdf_joint, limits.limits))
					limits.has_limits = true;
				if (joint_limits_interface::getSoftJointLimits(urdf_joint, limits.soft_limits))
					limits.has_soft_limits = true;
			}
		}
		// Get limits from the parameter server.
		if (joint_limits_interface::getJointLimits(joint_name, joint_limit_nh, limits.limits))
			limits.has_limits = true;
		// TODO : get soft limits from parameter server  --- can be got from yaml

		if (!limits.has_limits)
			return;

		if (*joint_type == urdf::Joint::UNKNOWN)
		{
			// Infer the joint type.

			if (limits.limits.has_position_limits)
			{
				*joint_type = urdf::Joint::REVOLUTE;
			}
			else
			{
				if (limits.limits.angle_wraparound)
					*joint_type = urdf::Joint::CONTINUOUS;
				else
					*joint_type = urdf::Joint::PRISMATIC;
			}
		}

	}


	// Get Transmissions from the URDF
	bool ARobotHW::ParseTransmissionsFromURDF(const std::string& urdf_string)
	{
		transmission_interface::TransmissionParser::parse(urdf_string, transmissions_);
		return true;
	}

	// private
	void ARobotHW::setUpdateFlag (const std_msgs::Bool value)
	{
		this->checkFlag = value.data;
	}

	// services
	bool ARobotHW::selfDetectionSrv(arobot_hw::SelfDetection::Request &req,
			arobot_hw::SelfDetection::Response &resp)
	{
		// lock services
		ROS_DEBUG("self detection service called");
		boost::mutex::scoped_lock guard(services_lock_);
		ROS_DEBUG("self detection service locked");


		ROS_DEBUG("self detection service finished");
		return true;

	}

	bool ARobotHW::driverDetectionSrv(arobot_hw::DriverDetection::Request &req,
			arobot_hw::DriverDetection::Response &resp)
	{
		// lock services
		ROS_DEBUG("drivers detection service called");
		boost::mutex::scoped_lock guard(services_lock_);
		ROS_DEBUG("drivers detection service locked");


		ROS_DEBUG("drivers detection service finished");
		return true;

	}
	bool ARobotHW::actuatorDetectionSrv(arobot_hw::ActuatorDetection::Request &req,
			arobot_hw::ActuatorDetection::Response &resp)
	{
		// lock services
		ROS_DEBUG("actuators detection service called");
		boost::mutex::scoped_lock guard(services_lock_);
		ROS_DEBUG("actuators detection service locked");


		ROS_DEBUG("actuators detection service finished");
		return true;

	}
	bool ARobotHW::sensorDetectionSrv(arobot_hw::SensorDetection::Request &req,
			arobot_hw::SensorDetection::Response &resp)
	{
		// lock services
		ROS_DEBUG("sensors detection service called");
		boost::mutex::scoped_lock guard(services_lock_);
		ROS_DEBUG("sensors detection service locked");

		ROS_DEBUG("sensors detection service finished");
		return true;

	}

	bool ARobotHW::sensorCalibrationSrv(arobot_hw::SensorCalibration::Request &req,
										arobot_hw::SensorCalibration::Response &resp)
	{
				// lock services
		ROS_DEBUG("sensor calibration service called");
		boost::mutex::scoped_lock guard(services_lock_);
		ROS_DEBUG("sensor calibration service locked");

		#ifdef ECAT
        using namespace artrobot::ecat;
		// ecrt_master_receive(EcatAdmin::master);
		// ecrt_domain_process(EcatAdmin::domain1);
        #endif

		if ( req.command ==  "left_ft") {
        #ifdef ECAT
			ROS_INFO("left ft calibration");
            EC_WRITE_U32(EcatAdmin::domain1_pd + PdoRegTable::regs[EcatAdmin::ft_dict[1]->get_r_pdo_reg_index(6)], 2);

            ecrt_domain_queue(EcatAdmin::domain1);
            ecrt_master_send(EcatAdmin::master);

		    usleep(10000);

			EC_WRITE_U32(EcatAdmin::domain1_pd + PdoRegTable::regs[EcatAdmin::ft_dict[1]->get_r_pdo_reg_index(6)], 0);
			update_l_ft_zero = true;
        #endif
			resp.info = "left ft calibration";
		}
		else if(req.command == "right_ft") {
			ROS_INFO("right ft calibration");
		#ifdef ECAT
			EC_WRITE_U32(EcatAdmin::domain1_pd + PdoRegTable::regs[EcatAdmin::ft_dict[2]->get_r_pdo_reg_index(6)], 2);

            ecrt_domain_queue(EcatAdmin::domain1);
            ecrt_master_send(EcatAdmin::master);

			usleep(10000);
			EC_WRITE_U32(EcatAdmin::domain1_pd + PdoRegTable::regs[EcatAdmin::ft_dict[2]->get_r_pdo_reg_index(6)], 0);
			update_r_ft_zero = true;
		#endif
			resp.info = "right ft calibration";
		}
		else if(req.command == "all") {
			ROS_INFO("calibration all sensors");
		#ifdef ECAT
			EC_WRITE_U32(EcatAdmin::domain1_pd + PdoRegTable::regs[EcatAdmin::ft_dict[1]->get_r_pdo_reg_index(6)], 2);
			EC_WRITE_U32(EcatAdmin::domain1_pd + PdoRegTable::regs[EcatAdmin::ft_dict[2]->get_r_pdo_reg_index(6)], 2);

            ecrt_domain_queue(EcatAdmin::domain1);
            ecrt_master_send(EcatAdmin::master);

			usleep(10000);
			EC_WRITE_U32(EcatAdmin::domain1_pd + PdoRegTable::regs[EcatAdmin::ft_dict[1]->get_r_pdo_reg_index(6)], 0);
			EC_WRITE_U32(EcatAdmin::domain1_pd + PdoRegTable::regs[EcatAdmin::ft_dict[2]->get_r_pdo_reg_index(6)], 0);
			update_l_ft_zero = true;
			update_r_ft_zero = true;
		#endif
			resp.info = "calibration all sensors";
		}
		else {
			ROS_INFO("sensor calibration command List : left_ft, right_ft, all");
            resp.info = "sensor calibration command List : left_ft, right_ft, all";
		}

		usleep(10000);
		ROS_DEBUG("sensor calibration service finished");
		resp.ok = true;
		return true;
	}


	bool ARobotHW::actuatorControlSrv(arobot_hw::ActuatorControl::Request &req,
									 arobot_hw::ActuatorControl::Response &resp)
	{
#ifdef ECAT
        using namespace artrobot::ecat;
#endif
		auto getStateName = [](actuatorStates& s){
			switch(s){
				case actuatorStates::RUN:
					return "run";
				case actuatorStates::SHUTDOWN:
					return "shutdown";
				case actuatorStates::POWER:
					return "poweron";
				case actuatorStates::CalZERO:
					return "cal";
			}

		};
		// lock services
		ROS_DEBUG("actuator control service called");
		boost::mutex::scoped_lock guard(services_lock_);
		ROS_DEBUG("actuator control service locked");

		// shutdown the motors
		if ( req.command ==  "shutdown") {
			actSta_ = actuatorStates::SHUTDOWN;

#ifdef ECAT
			for(unsigned int j=0; j < n_dof_; j++) {
				EC_WRITE_U32(EcatAdmin::domain1_pd + PdoRegTable::regs[
						EcatAdmin::motor_dict[getEcatID(joint_names_[j])]->get_r_pdo_reg_index(2)], 0x170000);
			}

            ecrt_domain_queue(EcatAdmin::domain1);
            ecrt_master_send(EcatAdmin::master);

		    usleep(10000);

#endif
		    resp.ok = true;
			ROS_INFO("actuator ShutDown");
		}

		else if(req.command == "poweron") {
			ROS_INFO("actuator Poweron");

            if(actSta_ == actuatorStates::SHUTDOWN || actSta_ == actuatorStates::CalZERO)
            {
#ifdef ECAT
                for(unsigned int j=0; j < n_dof_; j++) {

                    double position = joint_position_[j];

                    ROS_DEBUG_NAMED("arobot_hw", "name-command:%s \t %f", joint_names_[j].c_str(), joint_position_command_[j]);

                    // map ros -> robot
                    position = position * getName2Cal(joint_names_[j], name_map_ros) * getName2Cal(joint_names_[j], name_map_robot);

                    EC_WRITE_U32(EcatAdmin::domain1_pd + PdoRegTable::regs[
                            EcatAdmin::motor_dict[getEcatID(joint_names_[j])]->get_r_pdo_reg_index(0)],
                            angle_cal_dict_[getEcatID(joint_names_[j])]->calc(position));
                }

                ecrt_domain_queue(EcatAdmin::domain1);
                ecrt_master_send(EcatAdmin::master);

                usleep(10000);


                for(unsigned int j=0; j < n_dof_; j++) {
                    ROS_DEBUG_NAMED("arobot_hw", "name-position:%s \t %f", joint_names_[j].c_str(), joint_position_[j]);
                    EC_WRITE_U32(EcatAdmin::domain1_pd + PdoRegTable::regs[
                            EcatAdmin::motor_dict[getEcatID(joint_names_[j])]->get_r_pdo_reg_index(2)], 0x1f0000);
                }

                ecrt_domain_queue(EcatAdmin::domain1);
                ecrt_master_send(EcatAdmin::master);

                usleep(10000);
#endif
			    actSta_ = actuatorStates::POWER;
		        resp.ok = true;
            }
            else{
                ROS_DEBUG_NAMED("arobot_hw", "Poweron the actuator only be able on Shutdown/CalZERO states");
			    resp.info = "Poweron the actuator only be able on Shutdown/CalZERO states";
		        resp.ok = false;
            }
		}

		// calibrate the motors
		else if(req.command == "cal") {
			ROS_INFO("actuator calibration");
            actuatorStates pre_sta = actSta_;
			actSta_ = actuatorStates::CalZERO;

            if(pre_sta == actuatorStates::POWER || pre_sta == actuatorStates::RUN)
            {

#ifdef ECAT
                for(auto &item: artrobot::ecat::EcatAdmin::motor_dict) {
                    int motor_no = static_cast<int>(item.first);
                    // The function is blocked internally.
                    int status_code = artrobot::ecat::EcatAdmin::init_motor_zero_position(motor_no);
                    ROS_INFO_STREAM(motor_no << " " << status_code);
                    usleep(10000);
                }
#endif
		        resp.ok = true;
            }
            else
            {
                ROS_DEBUG_NAMED("arobot_hw", "Cal the actuator only be able on Poweron/Run states");
			    resp.info = "Cal the actuator only be able on Poweron/Run states";
		        resp.ok = false;
            }
		}

		//  start the motors
		else if(req.command == "start") {
			ROS_INFO("actuator Start");

			e_stop_active_ = false;

            if(actSta_ == actuatorStates::SHUTDOWN || actSta_ == actuatorStates::CalZERO)
            {
#ifdef ECAT
                for(unsigned int j=0; j < n_dof_; j++) {

                    double position = joint_position_[j];

                    ROS_INFO_NAMED("arobot_hw", "name-command:%s \t %f", joint_names_[j].c_str(), joint_position_command_[j]);

                    // map ros -> robot
                    position = position * getName2Cal(joint_names_[j], name_map_ros) * getName2Cal(joint_names_[j], name_map_robot);

                    EC_WRITE_U32(EcatAdmin::domain1_pd + PdoRegTable::regs[
                            EcatAdmin::motor_dict[getEcatID(joint_names_[j])]->get_r_pdo_reg_index(0)],
                            angle_cal_dict_[getEcatID(joint_names_[j])]->calc(position));
                }

                ecrt_domain_queue(EcatAdmin::domain1);
                ecrt_master_send(EcatAdmin::master);

                usleep(10000);


                for(unsigned int j=0; j < n_dof_; j++) {
                    ROS_INFO_NAMED("arobot_hw", "name-position:%s \t %f", joint_names_[j].c_str(), joint_position_[j]);
                    EC_WRITE_U32(EcatAdmin::domain1_pd + PdoRegTable::regs[
                            EcatAdmin::motor_dict[getEcatID(joint_names_[j])]->get_r_pdo_reg_index(2)], 0x1f0000);
                }


                ecrt_domain_queue(EcatAdmin::domain1);
                ecrt_master_send(EcatAdmin::master);

                usleep(10000);
#endif
            }
			    actSta_ = actuatorStates::RUN;
		        resp.ok = true;
		}
            		//  start the motors
		else if(req.command == "stop") {
            if(actSta_ == actuatorStates::RUN)
            {
                ROS_INFO("actuator Stop");
                e_stop_active_ = true;
                actSta_ = actuatorStates::RUN; // The stop means pause actually.
		        resp.ok = true;
            }
            else
            {
                ROS_DEBUG_NAMED("arobot_hw", "Stop the actuator only be able on Run states");
                resp.info = "Poweron the actuator only be able on Run states";
		        resp.ok = false;

            }
		}
		else {
			ROS_INFO("actuator Command List : start, shutdown, poweron, cal");
			resp.info = "actuator Command List : start, shutdown, poweron,  cal, stop";
		    resp.ok = false;
		}

		ROS_DEBUG("actuator control service finished");
		resp.states = getStateName(this->actSta_);
		return true;
	}

	bool ARobotHW::initJointsMap(void)
	{
		std::string search_param_name;

		if (model_nh_.searchParam("ros_2_cal", search_param_name))
		{
			ROS_INFO_ONCE_NAMED("arobot_hw", "arobot hardware is initialing the ros-2-cal map ID table from [%s]", search_param_name.c_str());
			model_nh_.getParam(search_param_name, name_map_ros);
		}
		else
		{
			ROS_FATAL_NAMED("arobot_hw", "ros-2-cal table can not be found." );
			return false;
		}

		if (model_nh_.searchParam("robot_2_cal", search_param_name))
		{
			ROS_INFO_ONCE_NAMED("arobot_hw", "arobot hardware is initialing the robot-2-cal map ID table from [%s]", search_param_name.c_str());
			model_nh_.getParam(search_param_name, name_map_robot);
		}
		else
		{
			ROS_FATAL_NAMED("arobot_hw", "robot-2-cal table can not be found." );
			return false;
		}

		for(auto i:joint_names_)
			ROS_INFO_STREAM_NAMED("arobot_hw", i<<"\t robot:\t"<<getName2Cal(i,name_map_robot)
					<<"\t ros:\t"<<getName2Cal(i,name_map_ros));

		return true;

	}
	// std::unordered_map<std::string, uint32_t> name_to_id_dir;
	bool ARobotHW::initEcatID(void)
	{
		std::string search_param_name;
		std::vector<int> slave_no;
		const int offset = 9;
		if (model_nh_.searchParam("ethercat_slave_no", search_param_name))
		{
			ROS_INFO_ONCE_NAMED("arobot_hw", "arobot hardware is initialing the ECAT ID table from [%s]", search_param_name.c_str());
			model_nh_.getParam(search_param_name, slave_no);
		}
		else
		{
			ROS_FATAL_NAMED("arobot_hw", "ethercat_slave_no can not be found." );
			return false;
		}
		for(int i = 0 ; i != slave_no.size() -1; ++i)
		{
			if (model_nh_.searchParam("joints_name_slave_"+ std::to_string(slave_no[i]), search_param_name))
			{
				std::vector<std::string> jlist;
				ROS_INFO_ONCE_NAMED("arobot_hw", "arobot hardware is initialing the ECAT ID table from [%s]", search_param_name.c_str());
				model_nh_.getParam(search_param_name, jlist);
				for(int j = 0;  j != jlist.size(); ++j)
					name_to_id_dir[jlist[j]] = offset*i + j + 1;
			}
			else
			{
			ROS_FATAL_NAMED("arobot_hw", "joints_name_slave_%d can not be found.", i);
				return false;
			}
		}

		assert(getEcatID("l_hip_pitch") == 12);
		assert(getEcatID("waist_pitch") == 8);
		assert(getEcatID("r_knee_pitch") == 22);

		return true;

	}

	bool ARobotHW::initEffortInfo(void)
	{
		std::string search_param_name;
		std::map<std::string, double> type_torque_map;
		std::map<std::string, double> type_current_map;
		std::map<std::string, double> i2t_param_map;

		if (model_nh_.searchParam("torque_const", search_param_name))
		{
			model_nh_.getParam(search_param_name, type_torque_map);
		}
		else
		{
			ROS_FATAL_NAMED("arobot_hw", "torque_const can not be found." );
			return false;
		}
		if (model_nh_.searchParam("nominal_current", search_param_name))
		{
			model_nh_.getParam(search_param_name, type_current_map);
		}
		else
		{
			ROS_FATAL_NAMED("arobot_hw", "nominal_current can not be found." );
			return false;
		}

		if (model_nh_.searchParam("i2t_param", search_param_name))
		{
			model_nh_.getParam(search_param_name, i2t_param_map);
		}
        else
		{
			ROS_FATAL_NAMED("arobot_hw", "i2t_param can not be found." );
			return false;
		}

		if (!model_nh_.searchParam("joint_motor_info", search_param_name))
		{
			ROS_FATAL_NAMED("arobot_hw", "joint_motor_info can not be found." );
			return false;
		}

		// joint_i2t_limit_.resize(n_dof_);
		for(int i = 0; i != n_dof_; ++i) {

			std::vector<std::string> info;

			if(!model_nh_.getParam(search_param_name + "/" + joint_names_[i], info))
			{
				ROS_FATAL_NAMED("arobot_hw", "joint info can not be found %s.", (search_param_name + "/" + joint_names_[i]).c_str());
			}

			joint_effort_coefficent_[i] = type_torque_map[info[0]] * atof(info[1].c_str());

			// i2t limit
			joint_i2t_limit_[i].name = joint_names_[i];
			joint_i2t_limit_[i].window = i2t_param_map["duration"];
			joint_i2t_limit_[i].nominal_current = type_current_map[info[0]];
			double c = i2t_param_map["multiple"] * joint_i2t_limit_[i].nominal_current;
			joint_i2t_limit_[i].limit = (c * c - joint_i2t_limit_[i].nominal_current * joint_i2t_limit_[i].nominal_current) * joint_i2t_limit_[i].window;
		}

		return true;
	}

	uint32_t ARobotHW::getEcatID(const std::string & str)
	{
		if(name_to_id_dir.find(str) != name_to_id_dir.end())
			return name_to_id_dir[str];
		else
			return 0;
	}

	int ARobotHW::getName2Cal(const std::string & str, std::map<std::string, int> & name_map)
	{
		if(name_map.find(str) != name_map.end())
			return name_map[str];
		else
			return 0;
	}


	double ARobotHW::adc2current(int value)
	{
        return 16.5*(double)value/4096.0;
	}

}
