#include "servo.h"
#include "motor_control.h"

#include <chrono>
#include <thread>


using namespace xrg_robot;
using namespace std::literals;

int main()
{
    servo_control servo;
    motor_control motor;

    // park manipulator
    servo.set_angle(3, 90);
    std::this_thread::sleep_for(300ms);
    servo.set_angle(1, 180);
    servo.set_angle(2, 10);
    std::this_thread::sleep_for(500ms);
    servo.set_angle(2, -10);
    std::this_thread::sleep_for(200ms);
    servo.save_as_default();

    // catch
    servo.set_angle(3, 180);
    servo.set_angle(2, 30);
    std::this_thread::sleep_for(300ms);
    servo.set_angle(4, 35);
    std::this_thread::sleep_for(100ms);
    servo.set_angle(1, 25);
    servo.set_angle(2, 50);
    std::this_thread::sleep_for(500ms);

    // grap
    servo.set_angle(3, 90);
    servo.set_angle(4, 75);
    std::this_thread::sleep_for(200ms);
    servo.set_angle(3, 0);
    servo.set_angle(4, 90);
    std::this_thread::sleep_for(200ms);

    // carry
    servo.set_angle(3, 180);
    std::this_thread::sleep_for(300ms);
    servo.set_angle(1, 180);
    servo.set_angle(2, 60);
    std::this_thread::sleep_for(500ms);
    motor.set_speed(1, 1);
    std::this_thread::sleep_for(1500ms);
    motor.set_speed(0, 0);

    // drop
    servo.set_angle(2, 30);
    std::this_thread::sleep_for(200ms);
    servo.set_angle(1, 50);
    servo.set_angle(2, 80);
    std::this_thread::sleep_for(500ms);
    servo.set_angle(3, 180);
    servo.set_angle(4, 60);
    std::this_thread::sleep_for(500ms);

    // park and go back
    servo.set_angle(3, 90);
    std::this_thread::sleep_for(300ms);
    servo.set_angle(1, 180);
    servo.set_angle(2, 10);
    std::this_thread::sleep_for(500ms);
    servo.set_angle(2, -10);
    std::this_thread::sleep_for(200ms);
    motor.set_speed(-1, -1);
    std::this_thread::sleep_for(1500ms);
    motor.set_speed(0, 0);

    return 0;
}