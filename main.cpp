#include "efp.hpp"
#include "pigpio.h"
#include "impl.hpp"

using namespace efp;

constexpr double min_freq_hz = 100;
constexpr double max_freq_hz = 1000;
constexpr int freq_bin_num = 100;
constexpr int cycle_time_us = 1'000;
constexpr double cycle_per_bin = 500;

String dir_path = "/Users/chanwooahn/Documents/dev/cpp/rpi-motor-system-id";

int main(int argc, char const *argv[])
{
    signal(SIGINT, on_exit);

    if (gpioInitialise() < 0)
    {
        std::cout << "err: gpioInitialise" << std::endl;
        abort();
    }

    auto state_machine = StateMachine{17, 0, 2, cycle_time_us};

    auto encoder = Encoder::instance();
    auto dc_motor = DcMotor::instance();
    auto leds = Leds::instance();

    auto sw_1 = Switch<switch_1_gpio>::instance(
        to_function_pointer(
            [&]()
            {
                state_machine.traj_ = Traj::Red1;
                state_machine.start_time_us_ = now_us();
                leds(1);
            }));

    auto sw_2 = Switch<switch_2_gpio>::instance(
        to_function_pointer(
            [&]()
            {
                state_machine.traj_ = Traj::Green2;
                state_machine.start_time_us_ = now_us();
                leds(2);
            }));

    auto sw_3 = Switch<switch_3_gpio>::instance(
        to_function_pointer(
            [&]()
            {
                state_machine.traj_ = Traj::Yellow3;
                state_machine.start_time_us_ = now_us();
                leds(3);
            }));

    bool run = true;

    int next = now_us();

    while (run)
    {
        const int now = now_us();
        if (now > next)
        {
            const int pos = encoder();
            const auto voltage_v = state_machine(pos, now);
            dc_motor(voltage_v);

            next += cycle_time_us;
        }
    }

    const Writer writer{state_machine};
    writer.write(dir_path);

    return 0;
}
