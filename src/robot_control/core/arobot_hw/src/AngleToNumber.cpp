//
// Created by han on 16-12-13.
//

#include "AngleToNumber.h"

AngleToNumber::AngleToNumber(int actuator_no, int resolution_ratio, int reduction_ratio){
    actuator_no_ = actuator_no;
    resolution_ratio_ = resolution_ratio;
    reduction_ratio_ = reduction_ratio;
    actuator_type_ = ActuatorType::Motor;
}

AngleToNumber::AngleToNumber(int actuator_no){
    actuator_no_ = actuator_no;
    actuator_type_ = ActuatorType::Servo;
}

uint32_t AngleToNumber::calc(double angle){
    if(actuator_type_ == ActuatorType::Motor){
        double tmp = (angle*resolution_ratio_*reduction_ratio_*4)/(2*3.14159265);
        return static_cast<uint32_t>(tmp);
    }else{
        // 500 = 45000 / 90
        double real_val = angle * 100000 / 3.14159265;
        type_conv_.real_val = static_cast<int>(real_val);
        return type_conv_.trans_val;
    }
}

//work for motor only
double AngleToNumber::inv_calc(uint32_t response_value){
    if(actuator_type_ == ActuatorType::Motor){
        int tmp = static_cast<int>(response_value);
        return (tmp*(2*3.14159265)/(resolution_ratio_*reduction_ratio_*4));
    } else {
        int tmp = static_cast<int>(response_value);
        return tmp / 100000.0 * 3.14159265;
    }
}
