#include "servo.h"
#include "software_pwm.h"

#include <wiringPi.h>


using namespace xrg_robot;

int main()
{
    servo_control servo;
    software_pwm pwm(1000);
    servo.set_angle(1, 90);
    servo.set_angle(2, 90);

    pinMode(23, OUTPUT);
    pinMode(24, OUTPUT);
    pinMode(27, OUTPUT);

    pinMode(28, OUTPUT);
    pinMode(29, OUTPUT);
    pinMode(25, OUTPUT);

    pwm[23] = 0.5;
    pwm[28] = 0.5;
    pwm.update();

    digitalWrite(24, HIGH);
    digitalWrite(27, LOW);
    digitalWrite(29, HIGH);
    digitalWrite(25, LOW);

    delayMicroseconds(2000000);
    pwm[23] = 0.;
    pwm[28] = 0.;
    pwm.update();
    digitalWrite(24, LOW);
    digitalWrite(27, LOW);
    digitalWrite(29, LOW);
    digitalWrite(25, LOW);
    

    return 0;
}