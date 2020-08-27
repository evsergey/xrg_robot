#include "sonar.h"

#include "isr.h"

#include <mutex>
#include <wiringPi.h>

namespace xrg_robot
{
namespace impl
{
class sonar final
{
private:
    int _trig_pin;
    int _echo_pin;
    int _min_interval;
    bool _first_pulse;
    int _pulse_start_time;

    struct measurement_info
    {
        int receive_time;
        double distance;
        double velocity;
    };
    mutable std::mutex _measurement_mutex;
    measurement_info _last_measurement;

public:
    sonar(int trig_pin, int echo_pin, int frequency):
        _trig_pin(trig_pin),
        _echo_pin(echo_pin),
        _min_interval(frequency == 0 ? 0 : 1000000 / frequency),
        _first_pulse(true),
        _last_measurement{}
    {
        pinMode(trig_pin, OUTPUT);
        pinMode(echo_pin, INPUT);
        pullUpDnControl(echo_pin, PUD_UP);
    }

    template<class Proc>
    void start(const Proc& proc) const
    {
        set_handler(_echo_pin, proc);
        send_pulse();
    }

    void stop() const
    {
        set_handler(_echo_pin);
    }

    double get_distance() const
    {
        const int t = micros();
        std::unique_lock<std::mutex> lock(_measurement_mutex);
        return _last_measurement.distance + (t - _last_measurement.receive_time) * _last_measurement.velocity;
    }

    void echo_changed()
    {
        const bool up = digitalRead(_echo_pin) == HIGH;
        if(_first_pulse)
        {
            if(!up)
                return;
            _first_pulse = false;
        }
        const int t = micros();
        if(up)
        {
            _pulse_start_time = t;
            return;
        }
        const int duration = t - _pulse_start_time;
        measurement_info measurement{t, duration * 1.761149E-4 - 4.5640035E-3, 0.};
        {
            std::unique_lock<std::mutex> lock(_measurement_mutex);
            _last_measurement = measurement;
        }
        if(duration + 115 < _min_interval)
            delayMicroseconds(_min_interval - duration - 15);
        send_pulse();
    }

private:
    void send_pulse() const
    {
        digitalWrite(_trig_pin, HIGH);
        delayMicroseconds(15);
        digitalWrite(_trig_pin, LOW);
    }
};
}

sonar::sonar(int trig_pin, int echo_pin, int frequency):
    _pimpl(std::make_shared<impl::sonar>(trig_pin, echo_pin, frequency))
{
    _pimpl->start([wimpl = std::weak_ptr<impl::sonar>(_pimpl)]()
    {
        if(auto pimpl = wimpl.lock())
            pimpl->echo_changed();
    });
}

sonar::~sonar()
{
    _pimpl->stop();
}

double sonar::get_distance() const
{
    return _pimpl->get_distance();
}
}