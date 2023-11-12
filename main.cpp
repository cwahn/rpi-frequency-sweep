#include "efp.hpp"
#include "pigpio.h"
#include "impl.hpp"

using namespace efp;

constexpr double min_freq_hz = 100;
constexpr double max_freq_hz = 1000;
constexpr int freq_bin_num = 100;
constexpr int cycle_time_ns = 1'000'000;
constexpr double cycle_per_bin = 500;

String dir_path = "/Users/chanwooahn/Documents/dev/cpp/rpi-motor-system-id";

int main(int argc, char const *argv[])
{
    auto sm = StateMachine{
        min_freq_hz,
        max_freq_hz,
        freq_bin_num,
        cycle_time_ns,
        cycle_per_bin};

    if (gpioInitialise() < 0)
    {
        std::cout << "err: gpioInitialise" << std::endl;
        abort();
    }
    auto encoder = Encoder::instance();

    bool run = true;
    int next = now_ns();

    while (run)
    {
        const int now = now_ns();
        if (now > next)
        {
            const int pos = encoder.position();

            const auto maybe_cmd = sm.react(now, pos);

            maybe_cmd.match(
                [](double cmd)
                { dc_motor.command(cmd); },
                [](Nothing _)
                { run = false; });

            next += cycle_time_ns;
        }
    }

    gpioTerminate();

    const Writer writer{sm};
    writer.write(dir_path);

    return 0;
}
