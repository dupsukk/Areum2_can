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
constexpr long CONTROL_PERIOD =3'000'000;
constexpr int MaxID = 34;
constexpr int MOTOR_NUM = 16;

// PP 모드 파라미터 (임시값 - 실측 후 조정할 것). 그리퍼는 MIT 그대로 사용.
constexpr float PP_VEL_MAX   = 2.0f;    // rad/s
constexpr float PP_ACC       = 10.0f;   // rad/s^2 (매뉴얼 기본값)
constexpr float PP_CURRENT_LIM = 10.0f;  // A (임시값 - 토크 리밋 대신 전류 리밋 사용)

constexpr double INIT_POS_TOLERANCE = 20.0 * M_PI / 180.0;   // 시작 시 영점 허용 편차 (rad)
bool startup_fault = false;                                   // 초기 위치 편차 초과 시 main이 비정상 종료

constexpr bool is_gripper(int can_id) { return can_id == CAN_ID_LEFT_EE || can_id == CAN_ID_RIGHT_EE; }

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
            for (auto& motor : vec) {
                if (is_gripper(motor.can_id)) motor.write_operation_frame(0, 0, 0);
                else motor.write_pp_velocity(PP_VEL_MAX);   // PP 모터: 위치를 모르므로 이동 없이 상태 프레임(피드백)만 유도. 값은 init과 동일
            }
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

    // 초기 피드백(2π 보정 후)이 영점에서 20도 이상 벗어난 모터가 있으면 제어를 시작하지 않고 종료
    bool pose_fault = false;
    std::apply([&](auto&... vecs) {
        (..., [&](auto& vec) {
            for (auto& motor : vec) {
                double corr = motor.Feedback_param.pos.load(std::memory_order_relaxed) + motor.pos_offset;
                if (std::fabs(corr) > INIT_POS_TOLERANCE) {
                    std::cerr << "[ID 0x" << std::hex << motor.can_id << std::dec << "] 초기 위치 편차 "
                              << corr * 180.0 / M_PI << " deg > " << INIT_POS_TOLERANCE * 180.0 / M_PI << " deg\n";
                    pose_fault = true;
                }
            }
        }(vecs));
    }, AReuMii);

    if (pose_fault) {
        running = false;
        startup_fault = true;
        std::apply([](auto&... vecs) {
            (..., [](auto& vec) {
                for (auto& motor : vec) motor.stop_motor();
            }(vecs));
        }, AReuMii);
        return nullptr;
    }

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


        // 그리퍼를 뺀 나머지 축은 매 주기 전송
        std::apply([](auto&... vecs) {
            (..., [](auto& vec) {
                for (auto& motor : vec) {
                    if (is_gripper(motor.can_id)) motor.write_updated_operation_frame();   // 그리퍼: MIT
                    else motor.write_pp_position(motor.control_param.pos.load(std::memory_order_relaxed));   // 나머지: PP
                }
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


    ctrl_buf[SHM_INDEX_LEFT_ELBOW_PITCH].pos = 0.0;
    ctrl_buf[SHM_INDEX_RIGHT_ELBOW_PITCH].pos = 0.0;

    shm_ptr->write_ctrl(ctrl_buf);


    while(running){
        while (!shm_ptr->try_read_ctrl(ctrl_buf));  // torn read면 재시도

        std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_PITCH].control_param.pos.store(std::clamp(-ctrl_buf[SHM_INDEX_LEFT_SHOULDER_PITCH].pos, MIN_POS_LEFT_SHOULDER_PITCH, MAX_POS_LEFT_SHOULDER_PITCH), std::memory_order_relaxed);  // 라디안임.
        std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_ROLL].control_param.pos.store(std::clamp(ctrl_buf[SHM_INDEX_LEFT_SHOULDER_ROLL].pos, MIN_POS_LEFT_SHOULDER_ROLL, MAX_POS_LEFT_SHOULDER_ROLL), std::memory_order_relaxed);  // 나중에 하드코딩이 아니라 상수나 매크로로 바꿀 것.
        std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_YAW].control_param.pos.store(std::clamp(-ctrl_buf[SHM_INDEX_LEFT_SHOULDER_YAW].pos, MIN_POS_LEFT_SHOULDER_YAW, MAX_POS_LEFT_SHOULDER_YAW), std::memory_order_relaxed);
        std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_ELBOW_PITCH].control_param.pos.store(std::clamp(-ctrl_buf[SHM_INDEX_LEFT_ELBOW_PITCH].pos, MIN_POS_LEFT_ELBOW, MAX_POS_LEFT_ELBOW), std::memory_order_relaxed);

        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_ROLL].control_param.pos.store(std::clamp(-ctrl_buf[SHM_INDEX_LEFT_WRIST_ROLL].pos, MIN_POS_LEFT_WRIST_ROLL, MAX_POS_LEFT_WRIST_ROLL), std::memory_order_relaxed);  // 라디안임.
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_YAW].control_param.pos.store(std::clamp(ctrl_buf[SHM_INDEX_LEFT_WRIST_YAW].pos, MIN_POS_LEFT_WRIST_YAW, MAX_POS_LEFT_WRIST_YAW), std::memory_order_relaxed);  // 나중에 하드코딩이 아니라 상수나 매크로로 바꿀 것.
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_PITCH].control_param.pos.store(std::clamp(ctrl_buf[SHM_INDEX_LEFT_WRIST_PITCH].pos, MIN_POS_LEFT_WRIST_PITCH, MAX_POS_LEFT_WRIST_PITCH), std::memory_order_relaxed);
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_GRIPPER].control_param.pos.store(-ctrl_buf[SHM_INDEX_LEFT_GRIPPER].pos, std::memory_order_relaxed);


        // std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_PITCH].control_param.vel.store(-ctrl_buf[SHM_INDEX_LEFT_SHOULDER_PITCH].vel, std::memory_order_relaxed);  // 라디안임.
        // std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_ROLL].control_param.vel.store(ctrl_buf[SHM_INDEX_LEFT_SHOULDER_ROLL].vel,  std::memory_order_relaxed);  // 나중에 하드코딩이 아니라 상수나 매크로로 바꿀 것.
        // std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_YAW].control_param.vel.store(-ctrl_buf[SHM_INDEX_LEFT_SHOULDER_YAW].vel,  std::memory_order_relaxed);
        // std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_ELBOW_PITCH].control_param.vel.store(-ctrl_buf[SHM_INDEX_LEFT_ELBOW_PITCH].vel, std::memory_order_relaxed);

        // std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_ROLL].control_param.vel.store(-ctrl_buf[SHM_INDEX_LEFT_WRIST_ROLL].vel, std::memory_order_relaxed);  // 라디안임.
        // std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_YAW].control_param.vel.store(ctrl_buf[SHM_INDEX_LEFT_WRIST_YAW].vel, std::memory_order_relaxed);  // 나중에 하드코딩이 아니라 상수나 매크로로 바꿀 것.
        // std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_PITCH].control_param.vel.store(ctrl_buf[SHM_INDEX_LEFT_WRIST_PITCH].vel, std::memory_order_relaxed);
        // std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_GRIPPER].control_param.vel.store(-ctrl_buf[SHM_INDEX_LEFT_GRIPPER].vel, std::memory_order_relaxed);


        std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_PITCH].control_param.pos.store(std::clamp(ctrl_buf[SHM_INDEX_RIGHT_SHOULDER_PITCH].pos, MIN_POS_RIGHT_SHOULDER_PITCH, MAX_POS_RIGHT_SHOULDER_PITCH), std::memory_order_relaxed);  // 라디안임.
        std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_ROLL].control_param.pos.store(-std::clamp(ctrl_buf[SHM_INDEX_RIGHT_SHOULDER_ROLL].pos, MIN_POS_RIGHT_SHOULDER_ROLL, MAX_POS_RIGHT_SHOULDER_ROLL), std::memory_order_relaxed);  // 나중에 하드코딩이 아니라 상수나 매크로로 바꿀 것.
        std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_YAW].control_param.pos.store(std::clamp(ctrl_buf[SHM_INDEX_RIGHT_SHOULDER_YAW].pos, MIN_POS_RIGHT_SHOULDER_YAW, MAX_POS_RIGHT_SHOULDER_YAW), std::memory_order_relaxed);
        std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_ELBOW_PITCH].control_param.pos.store(std::clamp(ctrl_buf[SHM_INDEX_RIGHT_ELBOW_PITCH].pos, MIN_POS_RIGHT_ELBOW, MAX_POS_RIGHT_ELBOW), std::memory_order_relaxed);

        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_ROLL].control_param.pos.store(std::clamp(ctrl_buf[SHM_INDEX_RIGHT_WRIST_ROLL].pos, MIN_POS_RIGHT_WRIST_ROLL, MAX_POS_RIGHT_WRIST_ROLL), std::memory_order_relaxed);  // 라디안임.
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_YAW].control_param.pos.store(-std::clamp(ctrl_buf[SHM_INDEX_RIGHT_WRIST_YAW].pos, MIN_POS_RIGHT_WRIST_YAW, MAX_POS_RIGHT_WRIST_YAW), std::memory_order_relaxed);  // 나중에 하드코딩이 아니라 상수나 매크로로 바꿀 것.
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_PITCH].control_param.pos.store(-std::clamp(ctrl_buf[SHM_INDEX_RIGHT_WRIST_PITCH].pos, MIN_POS_RIGHT_WRIST_PITCH, MAX_POS_RIGHT_WRIST_PITCH), std::memory_order_relaxed);
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_GRIPPER].control_param.pos.store(ctrl_buf[SHM_INDEX_RIGHT_GRIPPER].pos, std::memory_order_relaxed);


        // std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_PITCH].control_param.vel.store(ctrl_buf[SHM_INDEX_RIGHT_SHOULDER_PITCH].vel, std::memory_order_relaxed);  // 라디안임.
        // std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_ROLL].control_param.vel.store(-ctrl_buf[SHM_INDEX_RIGHT_SHOULDER_ROLL].vel,  std::memory_order_relaxed);  // 나중에 하드코딩이 아니라 상수나 매크로로 바꿀 것.
        // std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_YAW].control_param.vel.store(ctrl_buf[SHM_INDEX_RIGHT_SHOULDER_YAW].vel,  std::memory_order_relaxed);
        // std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_ELBOW_PITCH].control_param.vel.store(ctrl_buf[SHM_INDEX_RIGHT_ELBOW_PITCH].vel, std::memory_order_relaxed);

        // std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_ROLL].control_param.vel.store(ctrl_buf[SHM_INDEX_RIGHT_WRIST_ROLL].vel, std::memory_order_relaxed);  // 라디안임.
        // std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_YAW].control_param.vel.store(-ctrl_buf[SHM_INDEX_RIGHT_WRIST_YAW].vel, std::memory_order_relaxed);  // 나중에 하드코딩이 아니라 상수나 매크로로 바꿀 것.
        // std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_PITCH].control_param.vel.store(-ctrl_buf[SHM_INDEX_RIGHT_WRIST_PITCH].vel, std::memory_order_relaxed);
        // std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_GRIPPER].control_param.vel.store(ctrl_buf[SHM_INDEX_RIGHT_GRIPPER].vel, std::memory_order_relaxed);


        fb_buf[SHM_INDEX_LEFT_SHOULDER_PITCH].pos = -std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_PITCH].Feedback_param.pos.load(std::memory_order_relaxed)-std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_PITCH].pos_offset;
        fb_buf[SHM_INDEX_LEFT_SHOULDER_ROLL].pos = std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_ROLL].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_ROLL].pos_offset;
        fb_buf[SHM_INDEX_LEFT_SHOULDER_YAW].pos = -std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_YAW].Feedback_param.pos.load(std::memory_order_relaxed)-std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_YAW].pos_offset;
        fb_buf[SHM_INDEX_LEFT_ELBOW_PITCH].pos = -std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_ELBOW_PITCH].Feedback_param.pos.load(std::memory_order_relaxed)-std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_ELBOW_PITCH].pos_offset;
        
        fb_buf[SHM_INDEX_LEFT_WRIST_ROLL].pos = -std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_ROLL].Feedback_param.pos.load(std::memory_order_relaxed)-std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_ROLL].pos_offset;
        fb_buf[SHM_INDEX_LEFT_WRIST_YAW].pos = std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_YAW].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_YAW].pos_offset;
        fb_buf[SHM_INDEX_LEFT_WRIST_PITCH].pos = std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_PITCH].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_PITCH].pos_offset;
        fb_buf[SHM_INDEX_LEFT_GRIPPER].pos = -std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_GRIPPER].Feedback_param.pos.load(std::memory_order_relaxed)-std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_GRIPPER].pos_offset;


        fb_buf[SHM_INDEX_LEFT_SHOULDER_PITCH].vel = -std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_PITCH].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_LEFT_SHOULDER_ROLL].vel = std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_ROLL].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_LEFT_SHOULDER_YAW].vel = -std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_YAW].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_LEFT_ELBOW_PITCH].vel = -std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_ELBOW_PITCH].Feedback_param.vel.load(std::memory_order_relaxed);
        
        fb_buf[SHM_INDEX_LEFT_WRIST_ROLL].vel = -std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_ROLL].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_LEFT_WRIST_YAW].vel = std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_YAW].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_LEFT_WRIST_PITCH].vel = std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_PITCH].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_LEFT_GRIPPER].vel = -std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_GRIPPER].Feedback_param.vel.load(std::memory_order_relaxed);


        fb_buf[SHM_INDEX_LEFT_SHOULDER_PITCH].torque = -std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_PITCH].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_LEFT_SHOULDER_ROLL].torque = std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_ROLL].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_LEFT_SHOULDER_YAW].torque = -std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_YAW].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_LEFT_ELBOW_PITCH].torque = -std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_ELBOW_PITCH].Feedback_param.torque.load(std::memory_order_relaxed);

        fb_buf[SHM_INDEX_LEFT_WRIST_ROLL].torque = -std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_ROLL].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_LEFT_WRIST_YAW].torque = std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_YAW].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_LEFT_WRIST_PITCH].torque = std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_PITCH].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_LEFT_GRIPPER].torque = -std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_GRIPPER].Feedback_param.torque.load(std::memory_order_relaxed);


        fb_buf[SHM_INDEX_RIGHT_SHOULDER_PITCH].pos = std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_PITCH].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_PITCH].pos_offset;
        fb_buf[SHM_INDEX_RIGHT_SHOULDER_ROLL].pos = -std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_ROLL].Feedback_param.pos.load(std::memory_order_relaxed)-std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_ROLL].pos_offset;
        fb_buf[SHM_INDEX_RIGHT_SHOULDER_YAW].pos = std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_YAW].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_YAW].pos_offset;
        fb_buf[SHM_INDEX_RIGHT_ELBOW_PITCH].pos = std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_ELBOW_PITCH].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_ELBOW_PITCH].pos_offset;
        
        fb_buf[SHM_INDEX_RIGHT_WRIST_ROLL].pos = std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_ROLL].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_ROLL].pos_offset;
        fb_buf[SHM_INDEX_RIGHT_WRIST_YAW].pos = -std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_YAW].Feedback_param.pos.load(std::memory_order_relaxed)-std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_YAW].pos_offset;
        fb_buf[SHM_INDEX_RIGHT_WRIST_PITCH].pos = -std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_PITCH].Feedback_param.pos.load(std::memory_order_relaxed)-std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_PITCH].pos_offset;
        fb_buf[SHM_INDEX_RIGHT_GRIPPER].pos = std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_GRIPPER].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_GRIPPER].pos_offset;


        fb_buf[SHM_INDEX_RIGHT_SHOULDER_PITCH].vel = std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_PITCH].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_RIGHT_SHOULDER_ROLL].vel = -std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_ROLL].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_RIGHT_SHOULDER_YAW].vel = std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_YAW].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_RIGHT_ELBOW_PITCH].vel = std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_ELBOW_PITCH].Feedback_param.vel.load(std::memory_order_relaxed);
        
        fb_buf[SHM_INDEX_RIGHT_WRIST_ROLL].vel = std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_ROLL].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_RIGHT_WRIST_YAW].vel = -std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_YAW].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_RIGHT_WRIST_PITCH].vel = -std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_PITCH].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_RIGHT_GRIPPER].vel = std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_GRIPPER].Feedback_param.vel.load(std::memory_order_relaxed);


        fb_buf[SHM_INDEX_RIGHT_SHOULDER_PITCH].torque = std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_PITCH].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_RIGHT_SHOULDER_ROLL].torque = -std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_ROLL].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_RIGHT_SHOULDER_YAW].torque = -std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_YAW].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_RIGHT_ELBOW_PITCH].torque = std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_ELBOW_PITCH].Feedback_param.torque.load(std::memory_order_relaxed);

        fb_buf[SHM_INDEX_RIGHT_WRIST_ROLL].torque = std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_ROLL].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_RIGHT_WRIST_YAW].torque = -std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_YAW].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_RIGHT_WRIST_PITCH].torque = -std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_PITCH].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[SHM_INDEX_RIGHT_GRIPPER].torque = std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_GRIPPER].Feedback_param.torque.load(std::memory_order_relaxed);


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
    std::get<EL05_Vec>(AReuMii).emplace_back(s1, CAN_ID_LEFT_EE);


    std::get<RS02_Vec>(AReuMii).emplace_back(s2, CAN_ID_RIGHT_SHOULDER_PITCH);
    std::get<RS02_Vec>(AReuMii).emplace_back(s2, CAN_ID_RIGHT_SHOULDER_ROLL);
    std::get<RS00_Vec>(AReuMii).emplace_back(s2, CAN_ID_RIGHT_SHOULDER_YAW);
    std::get<RS00_Vec>(AReuMii).emplace_back(s2, CAN_ID_RIGHT_ELBOW);

    std::get<EL05_Vec>(AReuMii).emplace_back(s2, CAN_ID_RIGHT_WRIST_ROLL);
    std::get<EL05_Vec>(AReuMii).emplace_back(s2, CAN_ID_RIGHT_WRIST_YAW);
    std::get<EL05_Vec>(AReuMii).emplace_back(s2, CAN_ID_RIGHT_WRIST_PITCH);
    std::get<EL05_Vec>(AReuMii).emplace_back(s2, CAN_ID_RIGHT_EE);


    std::apply([](auto&... vecs) {
        (..., [](auto& vec) {
            for (auto& motor : vec) {
                if (is_gripper(motor.can_id)) motor.init_motor_MIT(10, 10);
                else motor.init_motor_PP(PP_VEL_MAX, PP_ACC, PP_CURRENT_LIM);
            }
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

    if (startup_fault) return -1;
    return 0;
}
