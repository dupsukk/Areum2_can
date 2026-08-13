#include "RobstrideMotor.hpp"
#include "Rx_handler.hpp"
#include "Sharemem.hpp"
#include <signal.h>
#include <vector>
#include <iostream>

#include <fcntl.h>
#include <pthread.h>


const char* CAN_INTERFACE_0 = "can0";
const char* CAN_INTERFACE_1 = "can1";
bool running = true;
constexpr long CONTROL_PERIOD = 2'500'000;
constexpr int MaxID = 34;
constexpr int MOTOR_NUM = 16;

Motor_con AReuMii;


void signal_handler(int signum) { 
    running = false;
    std::apply([](auto&... vecs) {
        (..., [](auto& vec) {
            for (auto& motor : vec) {
                motor.write_operation_frame(0, 0, 0);
                motor.control_param.pos = 0;
                motor.control_param.Kp = 0;
                motor.control_param.Kd = 0;
                motor.stop_motor();//motor.clear_fault();usleep(100);
        }
        }(vecs));
    }, AReuMii);
}


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
        }, AReuMii);
        std::cout << "---\n";
    }
    return nullptr;
}

void* CAN_Comm_thread(void* arg) {  // TODO : 캘리브레이션을 위한 로직을 따로 뺄 것 그리고 뭐 캔 인터페이스 갯수 맞출 수 있게 탬플릿을 넣거나 
    struct sched_param param;
    param.sched_priority = 99;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);

    Rx_handler<MaxID> hRx(AReuMii);
    can_frame cf;
    auto& can_interface_vec = *static_cast<std::vector<int>*>(arg);

    for(int s : can_interface_vec){
        while (readframe(s, &cf));
    }

    std::apply([](auto&... vecs) {
        (..., [](auto& vec) {
            for (auto& motor : vec) motor.write_operation_frame(0, 0, 0);
        }(vecs));
    }, AReuMii);

    // 첫 사이클: 피드백 수신 후 모든 모터 offset 캘리브레이션
    RealTimeClock RTC;
    RTC.wait_next(CONTROL_PERIOD);

    for(int s : can_interface_vec){
        while (readframe(s, &cf)) {
            auto [id, err, p, v, t, tem] = hRx.parse_Rx_frame(&cf);
            if (!err) hRx.Write_Fb(id, p, v, t, tem);
        }
    }

    std::apply([](auto&... vecs) {
        (..., [](auto& vec) {
            for (auto& motor : vec) motor.calibrate();
        }(vecs));
    }, AReuMii);

    RTC.reset();
    while (running) {
        RTC.wait_next(CONTROL_PERIOD);

        for(int s : can_interface_vec){
            while (readframe(s, &cf)) {
                auto [id, err, p, v, t, tem] = hRx.parse_Rx_frame(&cf);
                if (err) [[unlikely]] {
                    std::cerr << id << " errorcode: " << err;
                    return nullptr;
                }
                hRx.Write_Fb(id, p, v, t, tem);
            }
        }


        std::apply([](auto&... vecs) {
            (..., [](auto& vec) {
                for (auto& motor : vec) motor.write_updated_operation_frame();
            }(vecs));
        }, AReuMii);
    }

    return NULL;
}

void* update_Control_params(void* args){

    Control_Shm<MOTOR_NUM> ctrl_shm(13563267);
    auto* shm_ptr = ctrl_shm.get();

    Control_param ctrl_buf[MOTOR_NUM];
    Feedback_Param fb_buf[MOTOR_NUM];

    std::get<RS02_Vec>(AReuMii)[0].control_param.Kp.store(3, std::memory_order_relaxed);  
    std::get<RS02_Vec>(AReuMii)[1].control_param.Kp.store(3, std::memory_order_relaxed);  
    std::get<RS00_Vec>(AReuMii)[0].control_param.Kp.store(3, std::memory_order_relaxed);  
    std::get<RS00_Vec>(AReuMii)[1].control_param.Kp.store(3, std::memory_order_relaxed);  

    std::get<EL05_Vec>(AReuMii)[0].control_param.Kp.store(3, std::memory_order_relaxed);  
    std::get<EL05_Vec>(AReuMii)[1].control_param.Kp.store(3, std::memory_order_relaxed);  
    std::get<EL05_Vec>(AReuMii)[2].control_param.Kp.store(3, std::memory_order_relaxed);  


    std::get<RS02_Vec>(AReuMii)[0].control_param.Kd.store(1, std::memory_order_relaxed);  
    std::get<RS02_Vec>(AReuMii)[1].control_param.Kd.store(1, std::memory_order_relaxed);  
    std::get<RS00_Vec>(AReuMii)[0].control_param.Kd.store(1, std::memory_order_relaxed);  
    std::get<RS00_Vec>(AReuMii)[1].control_param.Kd.store(1, std::memory_order_relaxed);  

    std::get<EL05_Vec>(AReuMii)[0].control_param.Kd.store(1, std::memory_order_relaxed);  
    std::get<EL05_Vec>(AReuMii)[1].control_param.Kd.store(1, std::memory_order_relaxed);  
    std::get<EL05_Vec>(AReuMii)[2].control_param.Kd.store(1, std::memory_order_relaxed);  
    

    while(running){
        while (!shm_ptr->try_read_ctrl(ctrl_buf));  // torn read면 재시도

        std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_PITCH].control_param.pos.store(std::clamp(ctrl_buf[SHM_INDEX_LEFT_SHOULDER_PITCH].pos, -2.0, 2.0), std::memory_order_relaxed);  // 라디안임.
        std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_ROLL].control_param.pos.store(std::clamp(ctrl_buf[SHM_INDEX_LEFT_SHOULDER_ROLL].pos, -2.0, 2.0), std::memory_order_relaxed);  // 나중에 하드코딩이 아니라 상수나 매크로로 바꿀 것.
        std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_YAW].control_param.pos.store(std::clamp(ctrl_buf[SHM_INDEX_LEFT_SHOULDER_YAW].pos, -2.0, 2.0), std::memory_order_relaxed);
        std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_ELBOW_PITCH].control_param.pos.store(std::clamp(ctrl_buf[SHM_INDEX_LEFT_ELBOW_PITCH].pos, -2.0, 2.0), std::memory_order_relaxed);

        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_ROLL].control_param.pos.store(std::clamp(ctrl_buf[SHM_INDEX_LEFT_WRIST_ROLL].pos, -2.0, 2.0), std::memory_order_relaxed);  // 라디안임.
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_YAW].control_param.pos.store(std::clamp(ctrl_buf[SHM_INDEX_LEFT_WRIST_YAW].pos, -2.0, 2.0), std::memory_order_relaxed);  // 나중에 하드코딩이 아니라 상수나 매크로로 바꿀 것.
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_PITCH].control_param.pos.store(std::clamp(ctrl_buf[SHM_INDEX_LEFT_WRIST_PITCH].pos, -2.0, 2.0), std::memory_order_relaxed);


        std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_PITCH].control_param.vel.store(ctrl_buf[SHM_INDEX_LEFT_SHOULDER_PITCH].vel, std::memory_order_relaxed);  // 라디안임.
        std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_ROLL].control_param.vel.store(ctrl_buf[SHM_INDEX_LEFT_SHOULDER_ROLL].vel,  std::memory_order_relaxed);  // 나중에 하드코딩이 아니라 상수나 매크로로 바꿀 것.
        std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_YAW].control_param.vel.store(ctrl_buf[SHM_INDEX_LEFT_SHOULDER_YAW].vel,  std::memory_order_relaxed);
        std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_ELBOW_PITCH].control_param.vel.store(ctrl_buf[SHM_INDEX_LEFT_ELBOW_PITCH].vel, std::memory_order_relaxed);

        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_ROLL].control_param.vel.store(ctrl_buf[SHM_INDEX_LEFT_WRIST_ROLL].vel, std::memory_order_relaxed);  // 라디안임.
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_YAW].control_param.vel.store(ctrl_buf[SHM_INDEX_LEFT_WRIST_YAW].vel, std::memory_order_relaxed);  // 나중에 하드코딩이 아니라 상수나 매크로로 바꿀 것.
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_PITCH].control_param.vel.store(ctrl_buf[SHM_INDEX_LEFT_WRIST_PITCH].vel, std::memory_order_relaxed);


        std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_PITCH].control_param.pos.store(std::clamp(ctrl_buf[SHM_INDEX_RIGHT_SHOULDER_PITCH].pos, -2.0, 2.0), std::memory_order_relaxed);  // 라디안임.
        std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_ROLL].control_param.pos.store(std::clamp(ctrl_buf[SHM_INDEX_RIGHT_SHOULDER_ROLL].pos, -2.0, 2.0), std::memory_order_relaxed);  // 나중에 하드코딩이 아니라 상수나 매크로로 바꿀 것.
        std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_YAW].control_param.pos.store(std::clamp(ctrl_buf[SHM_INDEX_RIGHT_SHOULDER_YAW].pos, -2.0, 2.0), std::memory_order_relaxed);
        std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_ELBOW_PITCH].control_param.pos.store(std::clamp(ctrl_buf[SHM_INDEX_RIGHT_ELBOW_PITCH].pos, -2.0, 2.0), std::memory_order_relaxed);

        std::get<EL05_Vec>(AReuMii)[3].control_param.pos.store(std::clamp(ctrl_buf[SHM_INDEX_RIGHT_WRIST_ROLL].pos, -2.0, 2.0), std::memory_order_relaxed);  // 라디안임.
        std::get<EL05_Vec>(AReuMii)[4].control_param.pos.store(std::clamp(ctrl_buf[SHM_INDEX_RIGHT_WRIST_YAW].pos, -2.0, 2.0), std::memory_order_relaxed);  // 나중에 하드코딩이 아니라 상수나 매크로로 바꿀 것.
        std::get<EL05_Vec>(AReuMii)[5].control_param.pos.store(std::clamp(ctrl_buf[SHM_INDEX_RIGHT_WRIST_PITCH].pos, -2.0, 2.0), std::memory_order_relaxed);


        std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_PITCH].control_param.vel.store(ctrl_buf[SHM_INDEX_RIGHT_SHOULDER_PITCH].vel, std::memory_order_relaxed);  // 라디안임.
        std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_ROLL].control_param.vel.store(ctrl_buf[SHM_INDEX_RIGHT_SHOULDER_ROLL].vel,  std::memory_order_relaxed);  // 나중에 하드코딩이 아니라 상수나 매크로로 바꿀 것.
        std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_YAW].control_param.vel.store(ctrl_buf[SHM_INDEX_RIGHT_SHOULDER_YAW].vel,  std::memory_order_relaxed);
        std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_ELBOW_PITCH].control_param.vel.store(ctrl_buf[SHM_INDEX_RIGHT_ELBOW_PITCH].vel, std::memory_order_relaxed);

        std::get<EL05_Vec>(AReuMii)[3].control_param.vel.store(ctrl_buf[SHM_INDEX_RIGHT_WRIST_ROLL].vel, std::memory_order_relaxed);  // 라디안임.
        std::get<EL05_Vec>(AReuMii)[4].control_param.vel.store(ctrl_buf[SHM_INDEX_RIGHT_WRIST_YAW].vel, std::memory_order_relaxed);  // 나중에 하드코딩이 아니라 상수나 매크로로 바꿀 것.
        std::get<EL05_Vec>(AReuMii)[5].control_param.vel.store(ctrl_buf[SHM_INDEX_RIGHT_WRIST_PITCH].vel, std::memory_order_relaxed);




        fb_buf[SHM_INDEX_LEFT_SHOULDER_PITCH].pos = std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_PITCH].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_PITCH].pos_offset;
        fb_buf[SHM_INDEX_LEFT_SHOULDER_ROLL].pos = std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_ROLL].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_ROLL].pos_offset;
        fb_buf[SHM_INDEX_LEFT_SHOULDER_YAW].pos = std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_YAW].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_YAW].pos_offset;
        fb_buf[SHM_INDEX_LEFT_ELBOW_PITCH].pos = std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_ELBOW_PITCH].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_ELBOW_PITCH].pos_offset;
        
        fb_buf[SHM_INDEX_LEFT_WRIST_ROLL].pos = std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_ROLL].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_ROLL].pos_offset;
        fb_buf[SHM_INDEX_LEFT_WRIST_YAW].pos = std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_YAW].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_YAW].pos_offset;
        fb_buf[SHM_INDEX_LEFT_WRIST_PITCH].pos = std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_PITCH].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_PITCH].pos_offset;


        fb_buf[SHM_INDEX_LEFT_SHOULDER_PITCH].vel = std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_PITCH].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_LEFT_SHOULDER_ROLL].vel = std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_ROLL].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_LEFT_SHOULDER_YAW].vel = std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_YAW].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_LEFT_ELBOW_PITCH].vel = std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_ELBOW_PITCH].Feedback_param.vel.load(std::memory_order_relaxed);
        
        fb_buf[SHM_INDEX_LEFT_WRIST_ROLL].vel = std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_ROLL].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_LEFT_WRIST_YAW].vel = std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_YAW].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_LEFT_WRIST_PITCH].vel = std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_PITCH].Feedback_param.vel.load(std::memory_order_relaxed);


        fb_buf[SHM_INDEX_LEFT_SHOULDER_PITCH].torque = std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_PITCH].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_LEFT_SHOULDER_ROLL].torque = std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_ROLL].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_LEFT_SHOULDER_YAW].torque = std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_YAW].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_LEFT_ELBOW_PITCH].torque = std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_ELBOW_PITCH].Feedback_param.torque.load(std::memory_order_relaxed);

        fb_buf[SHM_INDEX_LEFT_WRIST_ROLL].torque = std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_ROLL].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_LEFT_WRIST_YAW].torque = std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_YAW].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_LEFT_WRIST_PITCH].torque = std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_PITCH].Feedback_param.torque.load(std::memory_order_relaxed);


        fb_buf[SHM_INDEX_RIGHT_SHOULDER_PITCH].pos = std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_PITCH].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_PITCH].pos_offset;
        fb_buf[SHM_INDEX_RIGHT_SHOULDER_ROLL].pos = std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_ROLL].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_ROLL].pos_offset;
        fb_buf[SHM_INDEX_RIGHT_SHOULDER_YAW].pos = std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_YAW].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_YAW].pos_offset;
        fb_buf[SHM_INDEX_RIGHT_ELBOW_PITCH].pos = std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_ELBOW_PITCH].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_ELBOW_PITCH].pos_offset;
        
        fb_buf[SHM_INDEX_RIGHT_WRIST_ROLL].pos = std::get<EL05_Vec>(AReuMii)[3].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<EL05_Vec>(AReuMii)[3].pos_offset;
        fb_buf[SHM_INDEX_RIGHT_WRIST_YAW].pos = std::get<EL05_Vec>(AReuMii)[4].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<EL05_Vec>(AReuMii)[4].pos_offset;
        fb_buf[SHM_INDEX_RIGHT_WRIST_PITCH].pos = std::get<EL05_Vec>(AReuMii)[5].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<EL05_Vec>(AReuMii)[5].pos_offset;


        fb_buf[SHM_INDEX_RIGHT_SHOULDER_PITCH].vel = std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_PITCH].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_RIGHT_SHOULDER_ROLL].vel = std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_ROLL].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_RIGHT_SHOULDER_YAW].vel = std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_YAW].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_RIGHT_ELBOW_PITCH].vel = std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_ELBOW_PITCH].Feedback_param.vel.load(std::memory_order_relaxed);
        
        fb_buf[SHM_INDEX_RIGHT_WRIST_ROLL].vel = std::get<EL05_Vec>(AReuMii)[3].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_RIGHT_WRIST_YAW].vel = std::get<EL05_Vec>(AReuMii)[4].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_RIGHT_WRIST_PITCH].vel = std::get<EL05_Vec>(AReuMii)[5].Feedback_param.vel.load(std::memory_order_relaxed);


        fb_buf[SHM_INDEX_RIGHT_SHOULDER_PITCH].torque = std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_PITCH].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_RIGHT_SHOULDER_ROLL].torque = std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_ROLL].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_RIGHT_SHOULDER_YAW].torque = std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_YAW].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_RIGHT_ELBOW_PITCH].torque = std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_ELBOW_PITCH].Feedback_param.torque.load(std::memory_order_relaxed);

        fb_buf[SHM_INDEX_RIGHT_WRIST_ROLL].torque = std::get<EL05_Vec>(AReuMii)[3].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_RIGHT_WRIST_YAW].torque = std::get<EL05_Vec>(AReuMii)[4].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_RIGHT_WRIST_PITCH].torque = std::get<EL05_Vec>(AReuMii)[5].Feedback_param.torque.load(std::memory_order_relaxed);


        // fb_buf[0].temp = std::get<RS02_Vec>(AReuMii)[0].Feedback_param.temp.load(std::memory_order_relaxed);
        // fb_buf[1].temp = std::get<RS02_Vec>(AReuMii)[1].Feedback_param.temp.load(std::memory_order_relaxed);
        // fb_buf[2].temp = std::get<RS00_Vec>(AReuMii)[0].Feedback_param.temp.load(std::memory_order_relaxed);
        // fb_buf[3].temp = std::get<RS00_Vec>(AReuMii)[1].Feedback_param.temp.load(std::memory_order_relaxed);
        shm_ptr->write_fb(fb_buf);

       usleep(1000);
    }
    return nullptr;
}


int main() {

    signal(SIGINT, signal_handler);



    int s1 = init_can(CAN_INTERFACE_0);
    if (s1 < 0) return -1;

    int s2 = init_can(CAN_INTERFACE_1);
    if (s2 < 0) return -1;

    std::vector<int> can_interface = {s1 , s2};

    std::get<RS02_Vec>(AReuMii).emplace_back(s1, CAN_ID_LEFT_SHOULDER_PITCH);
    std::get<RS02_Vec>(AReuMii).emplace_back(s1, CAN_ID_LEFT_SHOULDER_ROLL);
    std::get<RS00_Vec>(AReuMii).emplace_back(s1, CAN_ID_LEFT_SHOULDER_YAW);
    std::get<RS00_Vec>(AReuMii).emplace_back(s1, CAN_ID_LEFT_ELBOW);

    std::get<EL05_Vec>(AReuMii).emplace_back(s1, CAN_ID_LEFT_WRIST_ROLL);
    std::get<EL05_Vec>(AReuMii).emplace_back(s1, CAN_ID_LEFT_WRIST_YAW);
    std::get<EL05_Vec>(AReuMii).emplace_back(s1, CAN_ID_LEFT_WRIST_PITCH);


    std::get<RS02_Vec>(AReuMii).emplace_back(s2, CAN_ID_RIGHT_SHOULDER_PITCH);
    std::get<RS02_Vec>(AReuMii).emplace_back(s2, CAN_ID_RIGHT_SHOULDER_ROLL);
    std::get<RS00_Vec>(AReuMii).emplace_back(s2, CAN_ID_RIGHT_SHOULDER_YAW);
    std::get<RS00_Vec>(AReuMii).emplace_back(s2, CAN_ID_RIGHT_ELBOW);

    std::get<EL05_Vec>(AReuMii).emplace_back(s2, CAN_ID_RIGHT_WRIST_ROLL);
    std::get<EL05_Vec>(AReuMii).emplace_back(s2, CAN_ID_RIGHT_WRIST_YAW);
    std::get<EL05_Vec>(AReuMii).emplace_back(s2, CAN_ID_RIGHT_WRIST_PITCH);


    std::apply([](auto&... vecs) {
        (..., [](auto& vec) {
            for (auto& motor : vec) motor.init_motor_MIT(10, 10);
        }(vecs));
    }, AReuMii);

    //여기서 스레드 생성
    pthread_t rt_t, print_t, shm_t;
    pthread_create(&rt_t,    NULL, CAN_Comm_thread,   &can_interface);
    pthread_create(&print_t, NULL, print_thread_func,  nullptr);  // 초반 확인용이라 나중에는 안쓰는 스레드임. 
    pthread_create(&shm_t, NULL, update_Control_params,  nullptr);



    pthread_join(rt_t,    nullptr);
    pthread_join(print_t, nullptr);
    pthread_join(shm_t , nullptr);

    return 0;
}
