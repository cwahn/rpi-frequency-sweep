#ifndef IMPL_HPP
#define IMPL_HPP

#include <chrono>
#include <mutex>

#include <pigpio.h>

using namespace efp;
using namespace std::chrono;

constexpr int switch_1_gpio = 10;
constexpr int switch_2_gpio = 11;
constexpr int switch_3_gpio = 12;

constexpr int encoder_a = 20;
constexpr int encoder_b = 21;

constexpr int motor_0 = 20;
constexpr int motor_1 = 21;

constexpr int led_1_gpio = 13;
constexpr int led_2_gpio = 14;
constexpr int led_3_gpio = 15;

constexpr int count_per_rev = 216;

void on_exit(int signum)
{
    // Clean up pigpio resources
    gpioTerminate();
    printf("GPIO resources released, program terminated.\n");
}

enum class Traj
{
    None,
    Red1,
    Green2,
    Yellow3,
};

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

template <int gpio>
class Led
{
public:
    static Led &instance()
    {
        static led = Led{};
        return led;
    }

    void operator()(bool on)
    {
        if (on)
            gpioWrite(gpio, on);
        else
            gpioWrite(gpio, off);
    }

private:
    Led()
    {
        gpioSetMode(gpio, PI_OUTPUT);
    }
};

class Leds
{
public:
    static Leds &instance()
    {
        static Leds leds{};
        return leds;
    }

    void operator()(int led_num)
    {
        led_1_(led_num == 1);
        led_2_(led_num == 2);
        led_3_(led_num == 3);
    }

private:
    Leds()
        : led_1_{Led<led_1_gpio>::instance()},
          led_2_{Led<led_2_gpio>::instance()},
          led_3_{Led<led_3_gpio>::instance()}
    {
    }

    Led<led_1_gpio> &led_1_;
    Led<led_2_gpio> &led_2_;
    Led<led_3_gpio> &led_3_;
};

// HAL input

int now_us()
{
    static auto init_ = high_resolution_clock::now();
    const auto now_ = high_resolution_clock::now();
    return duration_cast<microseconds>(now_ - init_).count();
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
            enc_b == 0 ? ++enc_pos_ : --enc_pos_;
        else
            enc_b == 0 ? --enc_pos_ : ++enc_pos_;
    }

    void isr_b()
    {
        const int enc_a = gpioRead(encoder_a);
        const int enc_b = gpioRead(encoder_b);

        std::lock_guard<std::mutex> lock(m);

        if (enc_b == 1)
            enc_a == 0 ? --enc_pos_ : ++enc_pos_;
        else
            enc_a == 0 ? ++enc_pos_ : --enc_pos_;
    }

    int operator()
    {
        std::lock_guard<std::mutex> lock(m);
        return enc_pos_;
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
    int enc_pos_;
};

void isr_a()
{
    Encoder::instance().isr_a();
}
void isr_b()
{
    Encoder::instance().isr_b();
}

template <int gpio>
class Switch
{
public:
    static Switch &instance(void (*callback)())
    {
        static Switch s{callback};
        return s;
    }

private:
    Switch(void (*callback)())
    {
        const auto res = gpioSetISRFunc(gpio, FALLING_EDGE, 0, callback);

        if (res != 0)
            abort();
    };
};

// State machine

double rev_from_count(int pos_count)
{
    return (double)pos_count / (double)count_per_rev;
}

double trajectory1_rev(int time_us)
{
    const auto t_s = time_us * 0.000001;

    if (t_s < 3)
        return 2 * t_s;
    else if (t_s < 6)
        return -2 * t_s + 12;
    else if (t_s < 12)
        return 0;
    else if (t_s < 15)
        return 2 * t_s - 24;
    else
        return 0;
}

double trajectory2_rev(int time_us)
{
    const auto t_s = time_us * 0.000001;

    if (t_s < 5)
        return 0.2 * t_s;
    else if (t_s < 10)
        return -5 - 2 * cos(0.4 * M_PI * t_s) * sin(1.2 * M_PI * t_s);
    else if (t_x < 15)
        return 2 * (t_s - 12.5);
    else
        return 0;
}

double trajectory3_rev(int time_us)
{
    const auto t_s = time_us * 0.000001;

    if (t_s < 15)
        return exp(0.1 * t_s) * sin(M_PI * t_s) * cos(0.2 * M_PI * t_s);
    else
        return 0;
}

class StateMachine
{
public:
    StateMachine(double p, double i, double d, double time_delta_us)
        : traj_{Traj::None},
          time_delta_us_{time_delta_us},
          pid_{Pid{p, i, d, time_delta_us}}
    {
    }

    double operator()(int pos_count, int current_time_us)
    {
        if (traj_ != Traj::None)
        {
            const double ref_pos_rev;
            const time_us = current_time_us - start_time_;

            switch (traj_)
            {
            case Traj::Red1:
                ref_pos_rev = trajectory1_rev(time_us);
                break;
            case Traj::Green2:
                ref_pos_rev = trajectory2_rev(time_us);
                break;
            case Traj::Yellow:
                ref_pos_rev = trajectory3_rev(time_us);
                break;
            default:
                abort();
                break;
            }

            const auto error = ref_pos_rev - rev_from_count(pos_count);

            return pid_(error);
        }
        else
        {
            return 0;
        }
    }

private:
    Traj traj_;
    int start_time_;
    Pid pid_;
};

class Pid
{
public:
    Pid(double p, double i, double d, double time_delta_us)
        : p_(p), i_(i), d_(d), time_delta_us_(time_delta_us)
    {
    }

    double operator()(double e)
    {
        integral_e_ += e * time_delta_;
        const auto u = (p_ * e) + (i * integral_e_) + (d_ * (e - last_e_) / (time_delta_us_ * 1e-6));
        last_e_ = e;

        return u;
    }

private:
    double p_;
    double i_;
    double d_;
    double time_delta_us_;
    double last_e_;
    double integral_e_;
};

template <typename F>
double derivative(const F &f, int t_us, int delta_t_us)
{
    return (f(t_us + delta_t_us) - (t_us)) / (double)(delta_t_us * 0.000001);
}

template <typename As, typename Bs>
template itaes(const As &as, const Bs &refs, int delta_t_us)
{
    Vector<double> res{};
    double itae = 0;

    const auto inner = [](int i, auto a, auto ref)
    {
        itae += i * delta_t_us / 0.000001 * abs(a - ref);
        res.push_back(res);
    };
    for_each_with_index(inner, as, ref);

    return res;
}

template <typename As>
String to_csv(const As &as)
{
    const auto strings = map(std::itoa, as);
    const auto csv = join(", ", strings);
    return csv;
}

#endif