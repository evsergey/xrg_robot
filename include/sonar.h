#pragma once
#include <memory>

namespace xrg_robot
{
namespace impl{
    class sonar;
}
class sonar final
{
private:
    const std::shared_ptr<impl::sonar> _pimpl;

public:
    sonar(int trig_pin, int echo_pin, int frequency=0);
    ~sonar();

    double get_distance() const;
};
}
