#include <iostream>
#include <pigpio.h>
#include <chrono>
#include <thread>
#include <mutex>
#include <signal.h>
#include "efp.hpp"
#include <algorithm>
#include "server.hpp"

using namespace efp;
​
#define M1 23
#define M2 24
#define E1 25 // Encoder의 전원공급선
#define E2 20 // input
#define E3 21 // input
#define PU1 8
#define PU2 7
#define ENC2REDGEAR (12 * 180 * 4)
​
    // RPM = (60 * edge) / (t * PPR * 4) // edge : 단위 시간 당 들어온 신호 (rising, failing) 개수
    float initial_output = 200;
volatile sig_atomic_t terminate_program = 0;
void HandleSignal(int signal)
{
    // Signal handler for abrupt termination signals (e.g., Ctrl+C)
    terminate_program = 1;
}
void CleanupAndExit()
{
    // Clean up pigpio resources
    gpioTerminate();
    printf("GPIO resources released, program terminated.\n");
    exit(0);
}
class Encoder
{
public:
    void WriteA(int enc_a, int enc_b)
    {
        std::lock_guard<std::mutex> lock__(m);
        if (enc_a == 1)
        {
            if (enc_b == 0)
                enc_pos = enc_pos + 1;
            else
                enc_pos = enc_pos - 1;
        }
        else
        {
            if (enc_b == 0)
                enc_pos = enc_pos - 1;
            else
                enc_pos = enc_pos + 1;
        }
    }
    void WriteB(int enc_a, int enc_b)
    {
        std::lock_guard<std::mutex> lock__(m);
        if (enc_b == 1)
        {
            if (enc_a == 0)
                enc_pos = enc_pos - 1;
            else
                enc_pos = enc_pos + 1;
        }
        else
        {
            if (enc_a == 0)
                enc_pos = enc_pos + 1;
            else
                enc_pos = enc_pos - 1;
        }
    }
    double ReturnPWMValue()
    {
        float current_rpm = abs(Read());
        error = target_rpm - current_rpm;
        integral += error * 0.04;
        derivative = (error - lasterror) / 0.04;
        float output = initial_output + 13 * (kp * error + ki * integral + kd * derivative); // RPM 계산에서의 값이므로 수정 필요
        output = std::max(-1000.f, std::min(1000.f, output));
        pre_rpm = current_rpm;
        lasterror = error;
        SIGNAL_LOG(current_rpm, current_rpm);
        SIGNAL_LOG(error, error);
        SIGNAL_LOG(output, output);
        return output;
    }
    // PWM 값을 return

    double Read()
    {
        std::lock_guard<std::mutex> lock__(m);
        auto rpm = (60 * enc_pos) / (0.04 * ENC2REDGEAR);
        enc_pos = 0;
        return rpm;
    }
    ​ private : std::mutex m;
    int enc_pos{0}; // Pulse 개수를 저장하는 변수
    float pre_rpm = 0;
    float lasterror = 0;
    float error = 0;
    float integral = 0;
    float derivative = 0;
    float kp = 0.6;
    float ki = 1;
    float kd = 0.1;
    float target_rpm = 5;
};
Encoder EncoderPos;
void Con()
{
    Client x(std::string("tcp://192.168.0.8:5555"));
    x();
}
void FuncEncA(int gpio, int level, uint32_t tick)
{ // Encoder A에서 pin interrupt가 발생할 시 출력
    int enc_a = gpioRead(E2);
    int enc_b = gpioRead(E3);
    if (level == PI_TIMEOUT)
        // printf("Interrupt Watchdog timeout on GPIO %d\n", gpio);
        return;
    ​
        // printf("funcEncA A : %d, B : %d\n", EncA, EncB);
        EncoderPos.WriteA(enc_a, enc_b);
}
void FuncEncB(int gpio, int level, uint32_t tick)
{ // Encoder A에서 pin interrupt가 발생할 시 출력
    int enc_a = gpioRead(E2);
    int enc_b = gpioRead(E3);
    if (level == PI_TIMEOUT)
        // printf("Interrupt Watchdog timeout on GPIO %d\n", gpio);
        return;
    // printf("funcEncB A : %d, B : %d\n", EncA, EncB);
    EncoderPos.WriteB(enc_a, enc_b);
}

int main(int, char **)
{
    signal(SIGINT, HandleSignal);
    std::thread W(Con);
    std::cout << "Ready for gpiopwm test!" << std::endl;
    if (gpioInitialise() < 0)
    {
        std::cout << "err: gpioInitialise" << std::endl;
        exit(1);
    }
    SIGNAL_INIT_LOG_(SubMotor);
    gpioSetMode(M1, PI_OUTPUT);
    gpioSetMode(M2, PI_OUTPUT);
    gpioSetMode(E1, PI_OUTPUT);
    // gpioSetMode(E2, PI_INPUT);
    // gpioSetMode(E3, PI_INPUT);
    gpioSetPullUpDown(PU1, PI_PUD_UP);
    gpioSetPullUpDown(PU2, PI_PUD_UP);
    if (gpioSetISRFunc(E2, EITHER_EDGE, 10000, FuncEncA) != 0)
        exit(1);
    if (gpioSetISRFunc(E3, EITHER_EDGE, 10000, FuncEncB) != 0)
        exit(1);
    auto start = std::chrono::high_resolution_clock::now();
    auto start2 = std::chrono::high_resolution_clock::now();
    Vector<double> Con;
    gpioWrite(M1, 1);
    gpioWrite(M2, 1);
    gpioWrite(E1, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    gpioPWM(M1, initial_output);
    gpioPWM(M2, 0);
    gpioWrite(E1, 1);
    std::this_thread::sleep_for(std::chrono::seconds(5));
    while (!terminate_program)
    {
        const auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start2).count();
        if (duration > 40)
        {
            gpioPWM(M1, max(0, min(255, (int)EncoderPos.ReturnPWMValue())));
            // printf("output: %f",  currentPos);
            // con.push_back(currentPos);
            start2 = std::chrono::high_resolution_clock::now();
            SIGNAL_SYNC_LOG();
        }
        std::this_thread::sleep_for(std::chrono::microseconds(100)); // 조건 충족이 안될 때 while문이 너무 빨리 도는 현상 방지 (CPU 점유 방지)
        ​
    }
    if (terminate_program)
    {
        CleanupAndExit();
    }
    W.join();
    return 0;
}