#include "servo.h"

using namespace xrg_robot;

int main()
{
    servo_control servo;
    servo.set_angle(1, 90);
    servo.set_angle(2, 90);

    return 0;
}