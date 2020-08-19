#include "motor_control.h"

#include <cmath>
#include <initializer_list>
#include <wiringPi.h>

namespace xrg_robot
{
motor_control::motor_control(int left_power, int left_forward, int left_backward, int right_power, int right_forward, int right_backward)
    : _left_power(left_power)
    , _left_forward(left_forward)
    , _left_backward(left_backward)
    , _right_power(right_power)
    , _right_forward(right_forward)
    , _right_backward(right_backward)
    , _pwm(20)
{
    pinMode(_left_power, OUTPUT);
    pinMode(_left_forward, OUTPUT);
    pinMode(_left_backward, OUTPUT);
    pinMode(_right_power, OUTPUT);
    pinMode(_right_forward, OUTPUT);
    pinMode(_right_backward, OUTPUT);

    for(int pin: {_left_power, _left_forward, _left_backward, _right_power, _right_forward, _right_backward})
        digitalWrite(pin, LOW);
}

void motor_control::set_speed(double left, double right)
{
    bool need_update = false;
    auto set_power = [this, &need_update](int power_pin, int forward_pin, int backward_pin, double speed)
    {
        const double abs_speed = std::abs(speed);
        double& current_speed = _pwm[power_pin];
        need_update |= current_speed != abs_speed;
        current_speed = abs_speed;
        digitalWrite(forward_pin, speed > 0 ? HIGH : LOW);
        digitalWrite(backward_pin, speed < 0 ? HIGH : LOW);
    };

    set_power(_left_power, _left_forward, _left_backward, left);
    set_power(_right_power, _right_forward, _right_backward, right);
    if(need_update)
        _pwm.update();
}
}