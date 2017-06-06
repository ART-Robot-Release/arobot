//
// Created by han on 16-12-13.
//

#ifndef ROS_MASTER_NODE_ANGLETONUMBER_H
#define ROS_MASTER_NODE_ANGLETONUMBER_H

#include <cstdint>

enum class ActuatorType{
    Motor,
    Servo
};

union byte_4_angle {
    int32_t real_val;
    uint32_t trans_val;
};

class AngleToNumber {
public:
    AngleToNumber(int actuator_no, int resolution_ratio, int reduction_ratio);
    AngleToNumber(int actuator_no);
    ~AngleToNumber() = default;
    AngleToNumber(AngleToNumber&) = delete;

    uint32_t calc(double angle);

    double inv_calc(uint32_t response_value);

private:
    int actuator_no_;

    ActuatorType actuator_type_;
    int resolution_ratio_;
    int reduction_ratio_;

    byte_4_angle type_conv_;
};


#endif //ROS_MASTER_NODE_ANGLETONUMBER_H
