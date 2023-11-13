#ifndef IMPL_HPP
#define IMPL_HPP

#include <chrono>
#include <mutex>

#include <pigpio.h>

using namespace efp;
using namespace std::chrono;

constexpr int encoder_a = 20;
constexpr int encoder_b = 21;

constexpr int motor_0 = 20;
constexpr int motor_1 = 21;

// HAL output

class DcMotor
{
public:
    static DcMotor &instance()
    {
        static DcMotor motor{};
        return motor;
    }

    void opreator()(double cmd_v)
    {
        const double bound_cmd_v = bound_v(-3.3, 3.3, voltage);
        const int pwm_cmd = (int)(bound_cmd_v * 255. / 3.3);

        gpioPWM(motor_0, pwm_cmd >= 0 ? pwm_cmd : 0);
        gpioPWM(motor_1, pwm_cmd >= 0 ? 0 : -pwm_cmd);
    }

private:
    DcMotor()
    {
        gpioSetMode(motor_0, PI_OUTPUT);
        gpioSetMode(motor_1, PI_OUTPUT);
    }
};

// HAL input

int now_ns()
{
    static auto init_ = high_resolution_clock::now();
    const auto now_ = high_resolution_clock::now();
    return duration_cast<nanoseconds>(now_ - init_).count();
}

void isr_a();
void isr_b();

class Encoder
{
public:
    Encoder &instance()
    {
        static Encoder encoder{};
        return encoder;
    }

    void isr_a()
    {
        const int enc_a = gpioRead(encoder_a);
        const int enc_b = gpioRead(encoder_b);

        std::lock_guard<std::mutex> lock(m);

        if (enc_a == 1)
            enc_b == 0 ? ++enc_pos : --enc_pos;
        else
            enc_b == 0 ? --enc_pos : ++enc_pos;
    }

    void isr_b()
    {
        const int enc_a = gpioRead(encoder_a);
        const int enc_b = gpioRead(encoder_b);

        std::lock_guard<std::mutex> lock(m);

        if (enc_b == 1)
            enc_a == 0 ? --enc_pos : ++enc_pos;
        else
            enc_a == 0 ? ++enc_pos : --enc_pos;
    }

    int operator()
    {
        std::lock_guard<std::mutex> lock(m);
        return position_;
    }

private:
    Encoder()
    {
        if (gpioSetISRFunc(encoder_a, EITHER_EDGE, 0, isr_a) != 0)
            abort();

        if (gpioSetISRFunc(encoder_b, EITHER_EDGE, 0, isr_b) != 0)
            abort();
    }

    std::mutex m_;
    int position_;
};

void isr_a()
{
    Encoder::instance().isr_a();
}
void isr_b()
{
    Encoder::instance().isr_b();
}

// State machine

class StateMachine
{
public:
    StateMachine(
        double min_freq_hz,
        double max_freq_hz,
        int freq_bin_num,
        int cycle_time_ns,
        double cycle_per_bin)

        : min_freq_hz_(min_freq_hz_),
          max_freq_hz_(max_freq_hz_),
          freq_bin_num_(freq_bin_num),
          cycle_time_ns_(cycle_time_ns),
          cycle_per_bin_(cycle_per_bin),
          freq_duration_ns_(cycle_per_bin * cycle_time_ns),
          freq_bin_hz_((max_freq_hz_ - min_freq_hz_) / (double)freq_bin_num)
    {
    }

    Maybe<double> operator()(int now_ns, int encoder_pos)
    {
        positions_.push_back(encoder_pos);
        const auto motor_cmd = motor_cmd_(now_ns);
        const auto is_done = is_done_(now_ns);

        if (is_done)
            return Nothing{};
        else
            return motor_cmd;
    }

    Vector<int> &&get_pos_data()
    {
        return efp::move(positions_);
    }

    double min_freq_hz_;
    double max_freq_hz_;
    int freq_bin_num_;
    int cycle_time_ns_;
    double cycle_per_bin_;

    double freq_duration_ns_;
    double freq_bin_hz_;

    Vector<int> positions_;

private:
    int bin_(double now_ns)
    {
        return now_ns / (freq_duration_ns_);
    }

    double freq_hz_from_bin_(int bin)
    {
        return bin * freq_bin_hz_;
    }

    double freq_(double now_ns)
    {
        return freq_hz_from_bin_(bin_(now_ns));
    }

    bool is_done_(double now_ns)
    {
        return bin_(now_ns) >= freq_bin_num_;
    }

    double motor_cmd_(double now_ns)
    {
        return std::sin(2 * M_PI * freq_(now_ns) * now_ns);
    }
};

class Writer
{
public:
    Writer(const StateMachine &state_machine)
        : sm_{state_machine}
    {
    }

    bool write(String &dir_path)
    {
        const auto to_csv = [](const auto &ints)
        {
            const auto strings = map([](int i)
                                     { return format("{}", i); },
                                     ints);
            const auto csv = join(", ", strings);
            return csv;
        };

        const auto file_name = [&](auto freq_hz)
        {
            return format("{s}/{}_hz_{}_ns.csv", dir_path, freq_hz, sm_.freq_duration_ns_);
        };

        const auto write_bin_data = [&](const auto &data)
        {
            const auto csv = to_csv(data.positions);
            const auto file_name_ = file_name(data.freq_hz);

            auto file = File::open(file_name_, "w").value();
            file.write(csv);
        };

        const auto bin_datas = split_to_bin_();

        for_each(write_bin_data, bin_datas);
    }

private:
    struct BinData
    {
        double freq_hz;
        VectorView<int> positions;
    };

    Vector<BinData> split_to_bin_()
    {
        const auto idxs = from_function(sm_.freq_bin_num_, id<int>);

        const auto freq_hzs = map([&](int i)
                                  { return i * freq_bin_hz_; },
                                  idx);

        const auto slice_from_idx = [&](int idx)
        {
            const auto start = idx * sm_.cycle_per_bin_;
            const auto end = (1 + idx) * sm_.cycle_per_bin_;
            return slice(start, end, sm_.position_);
        };

        const auto slices = map(slice_from_idx, idxs);

        const auto bin_datas = map([&](double f_hz, const auto &slice)
                                   { return BinData{f_hz, slice}; },
                                   freq_hzs, slices);

        return bin_datas;
    }

    StateMachine sm_;
}

#endif