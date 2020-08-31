#include "sonar.h"

#include "isr.h"

#include <chrono>
#include <mutex>
#include <wiringPi.h>

constexpr double HALF_V_SOUND = 1.76127502E-4;
constexpr double DIST_OFFSET = 4.597691681E-3;
constexpr double DIST_STD = 0.0075;
constexpr double A_STD = 0.5;

constexpr double DIST_STD2 = DIST_STD * DIST_STD;
constexpr double A_STD2 = A_STD * A_STD;

namespace xrg_robot
{
namespace impl
{
class sonar final
{
private:
    using clock = std::chrono::steady_clock;
    using timestamp = clock::time_point;
    using ms_duration = std::chrono::microseconds;
    using s_duration = std::chrono::duration<double>;

    int _trig_pin;
    int _echo_pin;
    int _min_interval;
    bool _first_pulse;
    timestamp _pulse_start_time;

    struct measurement_info
    {
        timestamp receive_time;
        double distance;
        double velocity;
    };
    
    mutable std::mutex _measurement_mutex;
    measurement_info _last_measurement;
    double _p[4];

public:
    sonar(int trig_pin, int echo_pin, int frequency):
        _trig_pin(trig_pin),
        _echo_pin(echo_pin),
        _min_interval(frequency == 0 ? 0 : 1000000 / frequency),
        _first_pulse(true),
        _pulse_start_time(clock::now()),
        _last_measurement{},
        _p{-1., 0., 0., 0.}
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
        const timestamp t = clock::now();
        std::unique_lock<std::mutex> lock(_measurement_mutex);
        return _last_measurement.distance + std::chrono::duration_cast<s_duration>(t - _last_measurement.receive_time).count() * _last_measurement.velocity;
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
        const timestamp t = clock::now();
        if(up)
        {
            _pulse_start_time = t;
            return;
        }
        const auto duration = std::chrono::duration_cast<ms_duration>(t - _pulse_start_time);
        const auto r = duration.count() * HALF_V_SOUND + DIST_OFFSET;
        measurement_info measurement;
        if(_p[0] < 0)
        {
            measurement = {t, r, 0.};
            _p[0] = DIST_STD2;
            _p[3] = A_STD2;
        }else
        {
            const auto dt = std::chrono::duration_cast<s_duration>(t - _last_measurement.receive_time).count();
            // Kalman filter
            // 1) Prediction
            const auto r_pred = _last_measurement.distance + dt * _last_measurement.velocity;
            _p[0] += (_p[1] + _p[2] + _p[3] * dt) * dt + A_STD2 * dt*dt*dt*dt / 4;
            _p[1] += _p[3] * dt + A_STD2 * dt*dt*dt / 2;
            _p[2] += _p[3] * dt + A_STD2 * dt*dt*dt / 2;
            _p[3] += A_STD2 * dt*dt;
            // 2) Update
            const auto S = _p[0] + DIST_STD2;
            const auto y = r - r_pred;
            const auto w = 1 - _p[0] / S;
            _p[3] -= _p[1] * _p[2] / S;
            _p[0] *= w;
            _p[1] *= w;
            _p[2] *= w;
            const auto yS = y / S;
            measurement =
            {
                t,
                r_pred + _p[0] * yS,
                _last_measurement.velocity + _p[2] * yS
            };
        }
        {
            std::unique_lock<std::mutex> lock(_measurement_mutex);
            _last_measurement = measurement;
        }
        if(duration.count() + 115 < _min_interval)
            delayMicroseconds(_min_interval - duration.count() - 15);
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