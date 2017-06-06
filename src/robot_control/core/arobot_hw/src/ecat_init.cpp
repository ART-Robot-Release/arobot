//
// Created by han on 17-1-6.
//

#include "ecat_init.h"


void init_ecat_from_ros(ros::NodeHandle& nh, std::unordered_map<int, std::shared_ptr<AngleToNumber>>& angle_cal_dict, std::unordered_map<int, std::string>& motor_to_joint_name)
{
    std::vector<int> slave_no_list;
    nh.getParam("/ethercat_slave_no", slave_no_list);
    if(slave_no_list.size() != 4){
        ROS_INFO("ethercat_slave_no length error");
    }

    std::unordered_map<std::string, std::shared_ptr<std::vector<uint32_t>>> type_to_parameter;
    type_to_parameter["DCX_26"] = std::make_shared<std::vector<uint32_t>>();
    type_to_parameter["DCX_32"] = std::make_shared<std::vector<uint32_t>>();
    type_to_parameter["RE_40"] = std::make_shared<std::vector<uint32_t>>();
    type_to_parameter["NONE"] = std::make_shared<std::vector<uint32_t>>();

    std::vector<int> dcx_26_list;
    nh.getParam("/DCX_26", dcx_26_list);
    for(auto item: dcx_26_list){
        uint32_t tmp = static_cast<uint32_t>(item);
        type_to_parameter["DCX_26"]->push_back(tmp);
    }

    std::vector<int> dcx_32_list;
    nh.getParam("/DCX_32", dcx_32_list);
    for(auto item: dcx_32_list){
        uint32_t tmp = static_cast<uint32_t>(item);
        type_to_parameter["DCX_32"]->push_back(tmp);
    }

    std::vector<int> re_40_list;
    nh.getParam("/RE_40", re_40_list);
    for(auto item: re_40_list){
        uint32_t tmp = static_cast<uint32_t>(item);
        type_to_parameter["RE_40"]->push_back(tmp);
    }

    std::vector<int> none_list;
    nh.getParam("/NONE", none_list);
    for(auto item: re_40_list){
        uint32_t tmp = static_cast<uint32_t>(item);
        type_to_parameter["NONE"]->push_back(tmp);
    }

    std::unordered_map<std::string, int> motor_encoder_resolution;
    int tmp;
    nh.getParam("/resolution_ratio/DCX_26", tmp);
    motor_encoder_resolution["DCX_26"] = tmp;

    nh.getParam("/resolution_ratio/DCX_32", tmp);
    motor_encoder_resolution["DCX_32"] = tmp;

    nh.getParam("/resolution_ratio/RE_40", tmp);
    motor_encoder_resolution["RE_40"] = tmp;

    std::string name_prefix("joints_name_slave_");
    std::string type_prefix("motor_type_slave_");
    std::string i2t_prefix("i2t_slave_");

    int counter = 0;
    for (int i = 0; i < 3; ++i) {

        std::vector<std::string> joint_name_list;
        std::vector<std::string> motor_type_list;
        std::vector<int> reduction_list;
        std::vector<int> i2t_prefix_list;

        std::string joint_name = name_prefix + std::to_string(slave_no_list[i]);
        std::string motor_type = type_prefix + std::to_string(slave_no_list[i]);
        std::string i2t_name = i2t_prefix + std::to_string(slave_no_list[i]);

        std::string joint_name_all("/");
        joint_name_all += joint_name;

        nh.getParam(joint_name_all, joint_name_list);

        std::string motor_type_all("/");
        motor_type_all += motor_type;

        nh.getParam(motor_type_all, motor_type_list);

        std::string reduction_all("/reduction_ratio/");
        reduction_all += motor_type;

        nh.getParam(reduction_all, reduction_list);

        nh.getParam(i2t_name, i2t_prefix_list);



        artrobot::ecat::EcatAdmin::add_slaves(0, static_cast<uint16_t>(i), 0x00000009, static_cast<uint32_t>(slave_no_list[i]), joint_name_list.size(), 15);
        for (int j = 0; j < joint_name_list.size() ; ++j) {
            artrobot::ecat::EcatAdmin::add_item_motor((j+1+counter), 3, (0x6000 + j), (0x7000 + j));
            if(motor_type_list[j] == "NONE"){
                angle_cal_dict[((j+1+counter))] = std::make_shared<AngleToNumber>(((j+1+counter)));
            }else{
                angle_cal_dict[((j+1+counter))] = std::make_shared<AngleToNumber>(((j+1+counter)), motor_encoder_resolution[motor_type_list[j]], reduction_list[j]);
            }
            motor_to_joint_name[(j+1+counter)] = joint_name_list[j];
            std::shared_ptr<artrobot::ecat::ItemConfig> config_info_tmp = artrobot::ecat::EcatAdmin::slaves_dict[i]->get_item_config((j+1));

            for (int k = 0; k < 12 ; ++k) {
                config_info_tmp->pushback_config_values(type_to_parameter[motor_type_list[j]]->at(k));
            }
            if(motor_type_list[j] == "NONE") {
                config_info_tmp->pushback_config_values(0);
                config_info_tmp->pushback_config_values(0);

            }else{

                config_info_tmp->pushback_config_values(static_cast<uint32_t>(motor_encoder_resolution[motor_type_list[j]]));
                config_info_tmp->pushback_config_values(static_cast<uint32_t>(reduction_list[j]));
            }

            config_info_tmp->pushback_config_values(static_cast<uint32_t>(i2t_prefix_list[j]));

        }
        counter += joint_name_list.size();
    }

    artrobot::ecat::EcatAdmin::add_slaves(0, 3, 0x00000009, 0x00000002, 0, 0);

    artrobot::ecat::EcatAdmin::add_sensor(1, 1, 0x6000, 0x7000);

    artrobot::ecat::EcatAdmin::add_sensor(1, 2, 0x6003, 0x7003);

    artrobot::ecat::EcatAdmin::add_sensor(2, 1, 0x6006, 0x7006);


}

