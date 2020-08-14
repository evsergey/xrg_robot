#pragma once
#include <stdexcept>

namespace xrg_robot
{
class i2c_exception : public std::runtime_error
{
public:
    i2c_exception();
};
}
