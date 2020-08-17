#include "software_pwm.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <unordered_map>
#include <thread>
#include <vector>
#include <wiringPi.h>
namespace xrg_robot
{
namespace impl
{
class software_pwm final
{
private:
    enum action_type
    {
        pin_clear,
        pin_set,
        sleep
    };
    struct action
    {
        action_type type;
        int param;
    };

    const int _total;
    std::shared_ptr<std::vector<action>> _current_actions;
    std::atomic_bool _exit;
    std::thread _worker;

    void process_actions(const std::vector<action>& actions)
    {
        int slept = 0;
        for(auto& action: actions)
        {
            switch(action.type)
            {
                case action_type::pin_set:
                    digitalWrite(action.param, HIGH);
                    break;
                case action_type::pin_clear:
                    digitalWrite(action.param, LOW);
                    break;
                case action_type::sleep:
                    delayMicroseconds(action.param);
                    slept += action.param;
                    break;
                default:
                    break;
            }
        }
        return slept;
    }

    void thread_loop()
    {
        while(!_exit.load())
        {
            const auto actions = std::atomic_load(&_current_actions);
            int slept = actions ? process_actions(*actions) : 0;
            if(slept < _total)
                delayMicroseconds(_total - slept);
        }
    }

public:
    software_pwm(int frequency) :
        _total(1000000 / frequency),
        _exit(false),
        _worker(&software_pwm::thread_loop, this)
    {
        values.reserve(64);
    }

    ~software_pwm()
    {
        _exit.store(true);
        _worker.join();
    }

    void update()
    {
        std::vector<std::pair<int, int>> snapshot;
        snapshot.reserve(values.size());
        for(auto& pair: values)
            snapshot.emplace_back(pair.first, pair.second < 0 ? 0 : pair.second > 1 ? _total : static_cast<int>(std::round(pair.second * _total)));

        std::sort(snapshot.begin(), snapshot.end(), [](auto& a, auto& b){ return a.second < b.second; });
        std::vector<action> new_actions;
        new_actions.reserve(snapshot.size() * 3);
        for(auto& p: snapshot)
            new_actions.push_back(action{p.second == 0 ? action_type::pin_clear : action_type::pin_set, p.first});
        int current = 0;
        for(auto& p: snapshot)
            if(p.second > 0)
            {
                if(current < p.second)
                {
                    new_actions.push_back(action{action_type::sleep, p.second - current});
                    current = p.second;
                }
                if(p.second < _total)
                    new_actions.push_back(action{action_type::pin_clear, p.first});
            }
        std::atomic_store(&_current_actions, std::make_shared<decltype(new_actions)>(std::move(new_actions)));
    }

    std::unordered_map<int, double> values;
};
}

software_pwm::software_pwm(int frequency) :
    _pimpl(std::make_unique<impl::software_pwm>(frequency))
{
}

software_pwm::~software_pwm()
{
}

double software_pwm::operator[](int pin) const
{
    return _pimpl->values.at(pin);
}

double& software_pwm::operator[](int pin)
{
    return _pimpl->values[pin];
}

void software_pwm::update()
{
    _pimpl->update();
}
}