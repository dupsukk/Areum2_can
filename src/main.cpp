#include "RobstrideMotor.hpp"
#include "Rx_handler.hpp"
#include <signal.h>
#include <vector>
#include <iostream>

#include <fcntl.h>
#include <sys/mman.h>
#include <pthread.h>


const char* CAN_INTERFACE_0 = "can0";
bool running = true;
constexpr long CONTROL_PERIOD = 2000000;
constexpr int MaxID = 32;

constexpr int CAN_ID_LEFT_HIP_PITCH = 0x01;

Motor_con Leg;

void signal_handler(int signum) { running = false; }
RealTimeClock RTC;

void* print_thread_func(void*) {
    while (running) {
        usleep(500000);
        std::apply([](auto&... vecs) {
            (..., [](auto& vec) {
                for (const auto& motor : vec) {
                    std::cout
                        << "[ID 0x" << std::hex << motor.can_id << std::dec << "] "
                        << "pos="   << motor.Feedback_param.pos
                        << " corr=" << motor.Feedback_param.pos + motor.pos_offset
                        << " vel="  << motor.Feedback_param.vel
                        << " torq=" << motor.Feedback_param.torque
                        << " off="  << motor.pos_offset
                        << "\n";
                }
            }(vecs));
        }, Leg);
        std::cout << "---\n";
    }
    return nullptr;
}

void* CAN_Comm_thread(void* arg) {
    struct sched_param param;
    param.sched_priority = 99;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);

    Rx_handler<MaxID> hRx(Leg);
    can_frame cf;
    int s1 = *(int*)arg;

    while (readframe(s1, &cf));

    // 첫 사이클: 피드백 수신 후 모든 모터 offset 캘리브레이션
    RTC.reset();
    RTC.wait_next(CONTROL_PERIOD);
    while (readframe(s1, &cf)) {
        auto [id, err, p, v, t, tem] = hRx.parse_Rx_frame(&cf);
        if (!err) hRx.Write_Fb(id, p, v, t, tem);
    }
    std::apply([](auto&... vecs) {
        (..., [](auto& vec) {
            for (auto& motor : vec) motor.calibrate();
        }(vecs));
    }, Leg);

    RTC.reset();
    while (running) {
        RTC.wait_next(CONTROL_PERIOD);

        while (readframe(s1, &cf)) {
            auto [id, err, p, v, t, tem] = hRx.parse_Rx_frame(&cf);
            if (err) [[unlikely]] {
                std::cerr << id << " errorcode: " << err;
                return nullptr;
            }
            hRx.Write_Fb(id, p, v, t, tem);
        }

        std::apply([](auto&... vecs) {
            (..., [](auto& vec) {
                //for (auto& motor : vec) motor.write_updated_operation_frame();
                for (auto& motor : vec) motor.write_operation_frame(0, 0, 0);
            }(vecs));
        }, Leg);
    }

    return NULL;
}

int main() {

    signal(SIGINT, signal_handler);

    int s1 = init_can(CAN_INTERFACE_0);
    if (s1 < 0) return -1;

    /*
    std::get<RS04_Vec>(Leg).emplace_back(s1, 0x01);
    std::get<RS03_Vec>(Leg).emplace_back(s1, 0x02);
    std::get<RS03_Vec>(Leg).emplace_back(s1, 0x03);
    std::get<RS04_Vec>(Leg).emplace_back(s1, 0x04);
    std::get<RS06_Vec>(Leg).emplace_back(s1, 0x05);
    std::get<RS06_Vec>(Leg).emplace_back(s1, 0x06);
    */

    std::get<RS02_Vec>(Leg).emplace_back(s1, 0x01);
    std::get<RS02_Vec>(Leg).emplace_back(s1, 0x02);
    std::get<RS00_Vec>(Leg).emplace_back(s1, 0x03);
    std::get<RS00_Vec>(Leg).emplace_back(s1, 0x04);



    std::apply([](auto&... vecs) {
        (..., [](auto& vec) {
            for (auto& motor : vec) motor.init_motor_MIT(1, 1);
        }(vecs));
    }, Leg);

    //여기서 스레드 생성
    pthread_t rt_t, print_t;
    pthread_create(&rt_t,    NULL, CAN_Comm_thread,   &s1);
    pthread_create(&print_t, NULL, print_thread_func,  nullptr);

    /*
    int id, a;
    while (running) {
        std::cout << "motor id : ";
        std::cin >> id;
        std::cout << std::endl << "angle : ";
        std::cin >> a;
        std::cout << "\r\n\r\n";

        //여기에 모터 아이디와 각도를 넣으면
    }
    */

    pthread_join(rt_t,    nullptr);
    pthread_join(print_t, nullptr);

    return 0;
}
