#include "exceptions.h"
#include <cerrno>
#include <cstring>
#include <string>

namespace xrg_robot
{
i2c_exception::i2c_exception()
    : std::runtime_error(std::string("I2C error: ") + std::strerror(errno))
{
    
}
}
