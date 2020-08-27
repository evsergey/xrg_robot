#pragma once
#include <functional>

namespace xrg_robot
{
using interrupt_handler = std::function<void()>;
void set_handler(int pin, const interrupt_handler& new_handler = interrupt_handler{});
}
