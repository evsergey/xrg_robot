#pragma once
#include <memory>

namespace xrg_robot
{
namespace impl
{
    class software_pwm;
}
class software_pwm final
{
private:
    const std::unique_ptr<impl::software_pwm> _pimpl;
public:
    software_pwm(int frequency);
    ~software_pwm();
    double operator[](int pin) const;
    double& operator[](int pin);
    void update();
};
}
