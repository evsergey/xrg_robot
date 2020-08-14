#include "exceptions.h"
#include "servo.h"
#include "wiringPi.h"
#include "wiringPiI2C.h"

#include <initializer_list>

namespace xrg_robot
{
namespace
{
void write_bytes(int fd, std::initializer_list<int> list)
{
    for(int b: list)
    {
        if(wiringPiI2CWrite(fd, b))
            throw i2c_exception();
        delayMicroseconds(10);
    }
}
}

servo_control::servo_control()
{
    const static int unused = []()
    {
        return wiringPiSetup();
    }();
    _fd = wiringPiI2CSetup(0x17);
    if(_fd == -1)
        throw i2c_exception();
}

servo_control::~servo_control()
{
}

void servo_control::save_as_default() const
{
    write_bytes(_fd, {255, 17, 1, 255});
}

void servo_control::set_angle(int index, int angle) const
{
    write_bytes(_fd, {255, index, angle, 255});
}
}