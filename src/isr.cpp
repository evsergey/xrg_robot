#include "isr.h"

#include <array>
#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <utility>

#include <pthread.h>
#include <wiringPi.h>

namespace
{
constexpr int MAX_PIN = 64;
std::array<std::shared_ptr<xrg_robot::interrupt_handler>, MAX_PIN> handlers;
std::mutex thread_creation_mutex;

template<int pin>
void generic_handler()
{
    {
        auto& rhandler = handlers[pin];
        auto phandler = std::atomic_load(&rhandler);
        if(phandler)
            if(const auto& handler = *phandler)
            {
                handler();
                return;
            }
        std::unique_lock<std::mutex> lock(thread_creation_mutex);
        phandler = std::atomic_load(&rhandler);
        if(phandler && *phandler)
            return;
        std::atomic_store(&rhandler, decltype(phandler){});
    }
    pthread_exit(0);
}

template<int... pins>
constexpr auto generic_handlers_generator(std::integer_sequence<int, pins...>)
{
    return std::array<void(*)(), sizeof...(pins)>{ &generic_handler<pins>... };
}

constexpr auto generic_handlers = generic_handlers_generator(std::make_integer_sequence<int, MAX_PIN>{});
}

namespace xrg_robot
{
void set_handler(int pin, const interrupt_handler& new_handler)
{
    auto& rhandler = handlers[pin];
    auto phandler = std::atomic_load(&rhandler);
    if(!new_handler)
    {
        if(phandler && *phandler)
            std::atomic_store(&rhandler, std::make_shared<interrupt_handler>());
        return;
    }
    if(!phandler || *phandler)
        std::atomic_store(&rhandler, std::make_shared<interrupt_handler>(new_handler));
    else
    {
        std::unique_lock<std::mutex> lock(thread_creation_mutex);
        phandler = std::atomic_load(&rhandler);
        std::atomic_store(&rhandler, std::make_shared<interrupt_handler>(new_handler));
    }
    if(!phandler)
        wiringPiISR(pin, INT_EDGE_BOTH, generic_handlers[pin]);
}
}
