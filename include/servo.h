#pragma once

namespace xrg_robot
{
class servo_control final
{
private:
    int _fd;

public:
    servo_control();
    ~servo_control();

    void save_as_default() const;
    void set_angle(int index, int angle) const;
};

}
