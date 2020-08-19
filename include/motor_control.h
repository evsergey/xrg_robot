#pragma once
#include "software_pwm.h"

namespace xrg_robot
{
class motor_control
{
    int _left_power, _left_forward, _left_backward;
    int _right_power, _right_forward, _right_backward;

    software_pwm _pwm;

public:
    motor_control(
        int left_power = 23,
        int left_forward = 24,
        int left_backward = 27,
        int right_power = 28,
        int right_forward = 29,
        int right_backward = 25);

    void set_speed(double left, double right);
};
}