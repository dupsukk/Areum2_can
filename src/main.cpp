#include "RobstrideMotor.hpp"
#include "Rx_handler.hpp"
#include "Sharemem.hpp"
#include <signal.h>
#include <vector>
#include <iostream>

#include <fcntl.h>
#include <pthread.h>


const char* CAN_INTERFACE_0 = "can0";
bool running = true;
constexpr long CONTROL_PERIOD = 2'000'000;
constexpr int MaxID = 32;

Motor_con Leg;


void signal_handler(int signum) { running = false; }

void* print_thread_func(void*) {
    while (running) {
        usleep(500000);
        std::apply([](auto&... vecs) {
            (..., [](auto& vec) {
                for (const auto& motor : vec) {
                    std::cout
                        << "[ID 0x" << std::hex << motor.can_id << std::dec << "] "
                        << "pos="   << motor.Feedback_param.pos.load(std::memory_order_relaxed)
                        << " corr=" << motor.Feedback_param.pos.load(std::memory_order_relaxed) + motor.pos_offset
                        << " vel="  << motor.Feedback_param.vel.load(std::memory_order_relaxed)
                        << " torq=" << motor.Feedback_param.torque.load(std::memory_order_relaxed)
                        << " temp=" << motor.Feedback_param.temp.load(std::memory_order_relaxed)
                        << " off="  << motor.pos_offset
                        << "\n";
                }
            }(vecs));
        }, Leg);
        std::cout << "---\n";
    }
    return nullptr;
}

void* CAN_Comm_thread(void* arg) {  // TODO : 캘리브레이션을 위한 로직을 따로 뺄 것 그리고 뭐 캔 인터페이스 갯수 맞출 수 있게 탬플릿을 넣거나 
    struct sched_param param;
    param.sched_priority = 99;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);

    Rx_handler<MaxID> hRx(Leg);
    can_frame cf;
    int s1 = *(int*)arg;

    while (readframe(s1, &cf));
    std::apply([](auto&... vecs) {
        (..., [](auto& vec) {
            for (auto& motor : vec) motor.write_operation_frame(0, 0, 0);
        }(vecs));
    }, Leg);

    // 첫 사이클: 피드백 수신 후 모든 모터 offset 캘리브레이션
    RealTimeClock RTC;
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
                for (auto& motor : vec) motor.write_updated_operation_frame();
            }(vecs));
        }, Leg);
    }

    return NULL;
}

void* update_Control_params(void* args){

    Control_Shm<4> ctrl_shm(13563267);
    auto* shm_ptr = ctrl_shm.get();

    Control_param ctrl_buf[4];
    Feedback_Param fb_buf[4];
    

    while(running){
        while (!shm_ptr->try_read_ctrl(ctrl_buf));  // torn read면 재시도

        std::get<RS02_Vec>(Leg)[0].control_param.pos.store(std::clamp(ctrl_buf[0].pos, -2.0, 2.0), std::memory_order_relaxed);  // 라디안임.
        std::get<RS02_Vec>(Leg)[1].control_param.pos.store(std::clamp(ctrl_buf[1].pos, -2.0, 2.0), std::memory_order_relaxed);  // 나중에 하드코딩이 아니라 상수나 매크로로 바꿀 것.
        std::get<RS00_Vec>(Leg)[0].control_param.pos.store(std::clamp(ctrl_buf[2].pos, -2.0, 2.0), std::memory_order_relaxed);
        std::get<RS00_Vec>(Leg)[1].control_param.pos.store(std::clamp(ctrl_buf[3].pos, -2.0, 2.0), std::memory_order_relaxed);

        std::get<RS02_Vec>(Leg)[0].control_param.Kp.store(ctrl_buf[0].Kp, std::memory_order_relaxed);  
        std::get<RS02_Vec>(Leg)[1].control_param.Kp.store(ctrl_buf[1].Kp, std::memory_order_relaxed);  
        std::get<RS00_Vec>(Leg)[0].control_param.Kp.store(ctrl_buf[2].Kp, std::memory_order_relaxed);  
        std::get<RS00_Vec>(Leg)[1].control_param.Kp.store(ctrl_buf[3].Kp, std::memory_order_relaxed);  

        std::get<RS02_Vec>(Leg)[0].control_param.Kd.store(ctrl_buf[0].Kd, std::memory_order_relaxed);  
        std::get<RS02_Vec>(Leg)[1].control_param.Kd.store(ctrl_buf[1].Kd, std::memory_order_relaxed);  
        std::get<RS00_Vec>(Leg)[0].control_param.Kd.store(ctrl_buf[2].Kd, std::memory_order_relaxed);  
        std::get<RS00_Vec>(Leg)[1].control_param.Kd.store(ctrl_buf[3].Kd, std::memory_order_relaxed);  

        

        fb_buf[0].pos = std::get<RS02_Vec>(Leg)[0].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS02_Vec>(Leg)[0].pos_offset;
        fb_buf[1].pos = std::get<RS02_Vec>(Leg)[1].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS02_Vec>(Leg)[1].pos_offset;
        fb_buf[2].pos = std::get<RS00_Vec>(Leg)[0].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS00_Vec>(Leg)[0].pos_offset;
        fb_buf[3].pos = std::get<RS00_Vec>(Leg)[1].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS00_Vec>(Leg)[1].pos_offset;

        fb_buf[0].vel = std::get<RS02_Vec>(Leg)[0].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[1].vel = std::get<RS02_Vec>(Leg)[1].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[2].vel = std::get<RS00_Vec>(Leg)[0].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[3].vel = std::get<RS00_Vec>(Leg)[1].Feedback_param.vel.load(std::memory_order_relaxed);

        fb_buf[0].torque = std::get<RS02_Vec>(Leg)[0].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[1].torque = std::get<RS02_Vec>(Leg)[1].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[2].torque = std::get<RS00_Vec>(Leg)[0].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[3].torque = std::get<RS00_Vec>(Leg)[1].Feedback_param.torque.load(std::memory_order_relaxed);

        fb_buf[0].temp = std::get<RS02_Vec>(Leg)[0].Feedback_param.temp.load(std::memory_order_relaxed);
        fb_buf[1].temp = std::get<RS02_Vec>(Leg)[1].Feedback_param.temp.load(std::memory_order_relaxed);
        fb_buf[2].temp = std::get<RS00_Vec>(Leg)[0].Feedback_param.temp.load(std::memory_order_relaxed);
        fb_buf[3].temp = std::get<RS00_Vec>(Leg)[1].Feedback_param.temp.load(std::memory_order_relaxed);
        shm_ptr->write_fb(fb_buf);

       usleep(1000);
    }
    return nullptr;
}


int main() {

    signal(SIGINT, signal_handler);

    int s1 = init_can(CAN_INTERFACE_0);
    if (s1 < 0) return -1;

    std::get<RS02_Vec>(Leg).emplace_back(s1, CAN_ID_LEFT_SHOULDER_PITCH);
    std::get<RS02_Vec>(Leg).emplace_back(s1, CAN_ID_LEFT_SHOULDER_ROLL);
    std::get<RS00_Vec>(Leg).emplace_back(s1, CAN_ID_LEFT_SHOULDER_YAW);
    std::get<RS00_Vec>(Leg).emplace_back(s1, CAN_ID_LEFT_ELBOW);


    std::apply([](auto&... vecs) {
        (..., [](auto& vec) {
            for (auto& motor : vec) motor.init_motor_MIT(1, 3);
        }(vecs));
    }, Leg);

    //여기서 스레드 생성
    pthread_t rt_t, print_t, shm_t;
    pthread_create(&rt_t,    NULL, CAN_Comm_thread,   &s1);
    pthread_create(&print_t, NULL, print_thread_func,  nullptr);  // 초반 확인용이라 나중에는 안쓰는 스레드임. 
    pthread_create(&shm_t, NULL, update_Control_params,  nullptr);



    pthread_join(rt_t,    nullptr);
    pthread_join(print_t, nullptr);
    pthread_join(shm_t , nullptr);

    return 0;
}
