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

void* CAN_Comm_thread(void* arg) {  // TODO : 罹섎━釉뚮젅�댁뀡�� �꾪븳 濡쒖쭅�� �곕줈 類� 寃� 洹몃━怨� 萸� 罹� �명꽣�섏씠�� 媛�닔 留욎텧 �� �덇쾶 �ы뵆由우쓣 �ｊ굅�� 
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

    // 泥� �ъ씠��: �쇰뱶諛� �섏떊 �� 紐⑤뱺 紐⑦꽣 offset 罹섎━釉뚮젅�댁뀡
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

    // 洹몃━�쇰뒗 留� 二쇨린媛� �꾨땲�� 紐낅졊�� 諛붾�� �쒓컙�먮쭔 �꾨젅�꾩쓣 �좊┛��.
    struct GripperCmd { double pos, vel, Kp, Kd, ffTorque; };
    constexpr int GRIPPER_VEC_IDX[2] = {VEC_INDEX_LEFT_GRIPPER, VEC_INDEX_RIGHT_GRIPPER};
    GripperCmd last_gripper[2] = { {NAN,NAN,NAN,NAN,NAN}, {NAN,NAN,NAN,NAN,NAN} };  // 泥� �ъ씠�댁� 臾댁“嫄� 1�� �꾩넚

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


        // 洹몃━�쇰� 類� �섎㉧吏� 異뺤� 留� 二쇨린 �꾩넚
        std::apply([](auto&... vecs) {
            (..., [](auto& vec) {
                for (auto& motor : vec) {
                    if (motor.can_id == CAN_ID_LEFT_EE || motor.can_id == CAN_ID_RIGHT_EE) continue;
                    motor.write_updated_operation_frame();
                }
            }(vecs));
        }, AReuMii);

        // 洹몃━��: 吏곸쟾 �꾩넚媛믨낵 �ㅻ� �뚮쭔 �꾩넚
        bool gripper_sent = false;
        for (int i = 0; i < 2; ++i) {
            auto& g = std::get<EL05_Vec>(AReuMii)[GRIPPER_VEC_IDX[i]];
            GripperCmd cur{
                g.control_param.pos.load(std::memory_order_relaxed),
                g.control_param.vel.load(std::memory_order_relaxed),
                g.control_param.Kp.load(std::memory_order_relaxed),
                g.control_param.Kd.load(std::memory_order_relaxed),
                g.control_param.ffTorque.load(std::memory_order_relaxed)
            };
            if (cur.pos == last_gripper[i].pos && cur.vel == last_gripper[i].vel &&
                cur.Kp  == last_gripper[i].Kp  && cur.Kd  == last_gripper[i].Kd  &&
                cur.ffTorque == last_gripper[i].ffTorque) continue;

            g.write_updated_operation_frame();
            last_gripper[i] = cur;
            gripper_sent = true;
        }

        // 洹몃━�� �꾨젅�꾩씠 �섍컙 留뚰겮 二쇨린媛� 諛�由щ�濡� 湲곗� �쒓컖�� �ㅼ떆 �↔퀬 �ㅼ쓬 �ъ씠�대줈
        if (gripper_sent) RTC.reset();
    }

    return NULL;
}

void* update_Control_params(void* args){

    Control_Shm<MOTOR_NUM> ctrl_shm(13563267);
    auto* shm_ptr = ctrl_shm.get();

    Control_param ctrl_buf[MOTOR_NUM];
    Feedback_Param fb_buf[MOTOR_NUM];

    // �쇳뙏 Kp
    // std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_PITCH].control_param.Kp.store(10, std::memory_order_relaxed);
    // std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_ROLL].control_param.Kp.store(10, std::memory_order_relaxed);
    // std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_YAW].control_param.Kp.store(10, std::memory_order_relaxed);
    // std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_ELBOW_PITCH].control_param.Kp.store(10, std::memory_order_relaxed);

    // std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_ROLL].control_param.Kp.store(3, std::memory_order_relaxed);
    // std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_YAW].control_param.Kp.store(3, std::memory_order_relaxed);
    // std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_PITCH].control_param.Kp.store(3, std::memory_order_relaxed);
    // std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_GRIPPER].control_param.Kp.store(3, std::memory_order_relaxed);

    // // �ㅻⅨ�� Kp
    // std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_PITCH].control_param.Kp.store(10, std::memory_order_relaxed);
    // std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_ROLL].control_param.Kp.store(10, std::memory_order_relaxed);
    // std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_YAW].control_param.Kp.store(10, std::memory_order_relaxed);
    // std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_ELBOW_PITCH].control_param.Kp.store(10, std::memory_order_relaxed);

    // std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_ROLL].control_param.Kp.store(3, std::memory_order_relaxed);
    // std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_YAW].control_param.Kp.store(3, std::memory_order_relaxed);
    // std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_PITCH].control_param.Kp.store(3, std::memory_order_relaxed);
    // std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_GRIPPER].control_param.Kp.store(3, std::memory_order_relaxed);


    // // �쇳뙏 Kd
    // std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_PITCH].control_param.Kd.store(4, std::memory_order_relaxed);
    // std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_ROLL].control_param.Kd.store(4, std::memory_order_relaxed);
    // std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_YAW].control_param.Kd.store(4, std::memory_order_relaxed);
    // std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_ELBOW_PITCH].control_param.Kd.store(4, std::memory_order_relaxed);

    // std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_ROLL].control_param.Kd.store(1, std::memory_order_relaxed);
    // std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_YAW].control_param.Kd.store(1, std::memory_order_relaxed);
    // std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_PITCH].control_param.Kd.store(1, std::memory_order_relaxed);
    // std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_GRIPPER].control_param.Kd.store(1, std::memory_order_relaxed);

    // // �ㅻⅨ�� Kd
    // std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_PITCH].control_param.Kd.store(4, std::memory_order_relaxed);
    // std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_ROLL].control_param.Kd.store(4, std::memory_order_relaxed);
    // std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_YAW].control_param.Kd.store(4, std::memory_order_relaxed);
    // std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_ELBOW_PITCH].control_param.Kd.store(4, std::memory_order_relaxed);

    // std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_ROLL].control_param.Kd.store(1, std::memory_order_relaxed);
    // std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_YAW].control_param.Kd.store(1, std::memory_order_relaxed);
    // std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_PITCH].control_param.Kd.store(1, std::memory_order_relaxed);
    // std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_GRIPPER].control_param.Kd.store(1, std::memory_order_relaxed);


    ctrl_buf[SHM_INDEX_LEFT_ELBOW_PITCH].pos = 0.0;
    ctrl_buf[SHM_INDEX_RIGHT_ELBOW_PITCH].pos = 0.0;

    shm_ptr->write_ctrl(ctrl_buf);


    while(running){
        while (!shm_ptr->try_read_ctrl(ctrl_buf));  // torn read硫� �ъ떆��

        std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_PITCH].control_param.pos.store(std::clamp(-ctrl_buf[SHM_INDEX_LEFT_SHOULDER_PITCH].pos, -2.0, 2.0), std::memory_order_relaxed);  // �쇰뵒�덉엫.
        std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_ROLL].control_param.pos.store(std::clamp(ctrl_buf[SHM_INDEX_LEFT_SHOULDER_ROLL].pos, -2.0, 2.0), std::memory_order_relaxed);  // �섏쨷�� �섎뱶肄붾뵫�� �꾨땲�� �곸닔�� 留ㅽ겕濡쒕줈 諛붽� 寃�.
        std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_YAW].control_param.pos.store(std::clamp(-ctrl_buf[SHM_INDEX_LEFT_SHOULDER_YAW].pos, -2.0, 2.0), std::memory_order_relaxed);
        std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_ELBOW_PITCH].control_param.pos.store(std::clamp(-ctrl_buf[SHM_INDEX_LEFT_ELBOW_PITCH].pos, -2.0, 2.0), std::memory_order_relaxed);

        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_ROLL].control_param.pos.store(std::clamp(-ctrl_buf[SHM_INDEX_LEFT_WRIST_ROLL].pos, -2.0, 2.0), std::memory_order_relaxed);  // �쇰뵒�덉엫.
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_YAW].control_param.pos.store(std::clamp(ctrl_buf[SHM_INDEX_LEFT_WRIST_YAW].pos, -2.0, 2.0), std::memory_order_relaxed);  // �섏쨷�� �섎뱶肄붾뵫�� �꾨땲�� �곸닔�� 留ㅽ겕濡쒕줈 諛붽� 寃�.
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_PITCH].control_param.pos.store(std::clamp(ctrl_buf[SHM_INDEX_LEFT_WRIST_PITCH].pos, -2.0, 2.0), std::memory_order_relaxed);
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_GRIPPER].control_param.pos.store(std::clamp(-ctrl_buf[SHM_INDEX_LEFT_GRIPPER].pos, -2.0, 2.0), std::memory_order_relaxed);


        std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_PITCH].control_param.vel.store(-ctrl_buf[SHM_INDEX_LEFT_SHOULDER_PITCH].vel, std::memory_order_relaxed);  // �쇰뵒�덉엫.
        std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_ROLL].control_param.vel.store(ctrl_buf[SHM_INDEX_LEFT_SHOULDER_ROLL].vel,  std::memory_order_relaxed);  // �섏쨷�� �섎뱶肄붾뵫�� �꾨땲�� �곸닔�� 留ㅽ겕濡쒕줈 諛붽� 寃�.
        std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_YAW].control_param.vel.store(-ctrl_buf[SHM_INDEX_LEFT_SHOULDER_YAW].vel,  std::memory_order_relaxed);
        std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_ELBOW_PITCH].control_param.vel.store(-ctrl_buf[SHM_INDEX_LEFT_ELBOW_PITCH].vel, std::memory_order_relaxed);

        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_ROLL].control_param.vel.store(-ctrl_buf[SHM_INDEX_LEFT_WRIST_ROLL].vel, std::memory_order_relaxed);  // �쇰뵒�덉엫.
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_YAW].control_param.vel.store(ctrl_buf[SHM_INDEX_LEFT_WRIST_YAW].vel, std::memory_order_relaxed);  // �섏쨷�� �섎뱶肄붾뵫�� �꾨땲�� �곸닔�� 留ㅽ겕濡쒕줈 諛붽� 寃�.
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_PITCH].control_param.vel.store(ctrl_buf[SHM_INDEX_LEFT_WRIST_PITCH].vel, std::memory_order_relaxed);
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_GRIPPER].control_param.vel.store(-ctrl_buf[SHM_INDEX_LEFT_GRIPPER].vel, std::memory_order_relaxed);


        std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_PITCH].control_param.pos.store(std::clamp(ctrl_buf[SHM_INDEX_RIGHT_SHOULDER_PITCH].pos, -2.0, 2.0), std::memory_order_relaxed);  // �쇰뵒�덉엫.
        std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_ROLL].control_param.pos.store(-std::clamp(ctrl_buf[SHM_INDEX_RIGHT_SHOULDER_ROLL].pos, -2.0, 0.3), std::memory_order_relaxed);  // �섏쨷�� �섎뱶肄붾뵫�� �꾨땲�� �곸닔�� 留ㅽ겕濡쒕줈 諛붽� 寃�.
        std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_YAW].control_param.pos.store(std::clamp(ctrl_buf[SHM_INDEX_RIGHT_SHOULDER_YAW].pos, -1.5, 1.5), std::memory_order_relaxed);
        std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_ELBOW_PITCH].control_param.pos.store(std::clamp(ctrl_buf[SHM_INDEX_RIGHT_ELBOW_PITCH].pos, -2.0, 2.0), std::memory_order_relaxed);

        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_ROLL].control_param.pos.store(std::clamp(ctrl_buf[SHM_INDEX_RIGHT_WRIST_ROLL].pos, -2.0, 2.0), std::memory_order_relaxed);  // �쇰뵒�덉엫.
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_YAW].control_param.pos.store(-std::clamp(ctrl_buf[SHM_INDEX_RIGHT_WRIST_YAW].pos, -2.0, 2.0), std::memory_order_relaxed);  // �섏쨷�� �섎뱶肄붾뵫�� �꾨땲�� �곸닔�� 留ㅽ겕濡쒕줈 諛붽� 寃�.
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_PITCH].control_param.pos.store(-std::clamp(ctrl_buf[SHM_INDEX_RIGHT_WRIST_PITCH].pos, -2.0, 2.0), std::memory_order_relaxed);
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_GRIPPER].control_param.pos.store(std::clamp(ctrl_buf[SHM_INDEX_RIGHT_GRIPPER].pos, -2.0, 2.0), std::memory_order_relaxed);


        std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_PITCH].control_param.vel.store(ctrl_buf[SHM_INDEX_RIGHT_SHOULDER_PITCH].vel, std::memory_order_relaxed);  // �쇰뵒�덉엫.
        std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_ROLL].control_param.vel.store(-ctrl_buf[SHM_INDEX_RIGHT_SHOULDER_ROLL].vel,  std::memory_order_relaxed);  // �섏쨷�� �섎뱶肄붾뵫�� �꾨땲�� �곸닔�� 留ㅽ겕濡쒕줈 諛붽� 寃�.
        std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_YAW].control_param.vel.store(ctrl_buf[SHM_INDEX_RIGHT_SHOULDER_YAW].vel,  std::memory_order_relaxed);
        std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_ELBOW_PITCH].control_param.vel.store(ctrl_buf[SHM_INDEX_RIGHT_ELBOW_PITCH].vel, std::memory_order_relaxed);

        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_ROLL].control_param.vel.store(ctrl_buf[SHM_INDEX_RIGHT_WRIST_ROLL].vel, std::memory_order_relaxed);  // �쇰뵒�덉엫.
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_YAW].control_param.vel.store(-ctrl_buf[SHM_INDEX_RIGHT_WRIST_YAW].vel, std::memory_order_relaxed);  // �섏쨷�� �섎뱶肄붾뵫�� �꾨땲�� �곸닔�� 留ㅽ겕濡쒕줈 諛붽� 寃�.
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_PITCH].control_param.vel.store(-ctrl_buf[SHM_INDEX_RIGHT_WRIST_PITCH].vel, std::memory_order_relaxed);
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_GRIPPER].control_param.vel.store(ctrl_buf[SHM_INDEX_RIGHT_GRIPPER].vel, std::memory_order_relaxed);


        std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_PITCH].control_param.Kp.store(ctrl_buf[SHM_INDEX_LEFT_SHOULDER_PITCH].Kp, std::memory_order_relaxed);
        std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_ROLL].control_param.Kp.store(ctrl_buf[SHM_INDEX_LEFT_SHOULDER_ROLL].Kp, std::memory_order_relaxed);
        std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_YAW].control_param.Kp.store(ctrl_buf[SHM_INDEX_LEFT_SHOULDER_YAW].Kp, std::memory_order_relaxed);
        std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_ELBOW_PITCH].control_param.Kp.store(ctrl_buf[SHM_INDEX_LEFT_ELBOW_PITCH].Kp, std::memory_order_relaxed);

        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_ROLL].control_param.Kp.store(ctrl_buf[SHM_INDEX_LEFT_WRIST_ROLL].Kp, std::memory_order_relaxed);
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_YAW].control_param.Kp.store(ctrl_buf[SHM_INDEX_LEFT_WRIST_YAW].Kp, std::memory_order_relaxed);
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_PITCH].control_param.Kp.store(ctrl_buf[SHM_INDEX_LEFT_WRIST_PITCH].Kp, std::memory_order_relaxed);
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_GRIPPER].control_param.Kp.store(ctrl_buf[SHM_INDEX_LEFT_GRIPPER].Kp, std::memory_order_relaxed);

        std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_PITCH].control_param.Kp.store(ctrl_buf[SHM_INDEX_RIGHT_SHOULDER_PITCH].Kp, std::memory_order_relaxed);
        std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_ROLL].control_param.Kp.store(ctrl_buf[SHM_INDEX_RIGHT_SHOULDER_ROLL].Kp, std::memory_order_relaxed);
        std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_YAW].control_param.Kp.store(ctrl_buf[SHM_INDEX_RIGHT_SHOULDER_YAW].Kp, std::memory_order_relaxed);
        std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_ELBOW_PITCH].control_param.Kp.store(ctrl_buf[SHM_INDEX_RIGHT_ELBOW_PITCH].Kp, std::memory_order_relaxed);

        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_ROLL].control_param.Kp.store(ctrl_buf[SHM_INDEX_RIGHT_WRIST_ROLL].Kp, std::memory_order_relaxed);
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_YAW].control_param.Kp.store(ctrl_buf[SHM_INDEX_RIGHT_WRIST_YAW].Kp, std::memory_order_relaxed);
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_PITCH].control_param.Kp.store(ctrl_buf[SHM_INDEX_RIGHT_WRIST_PITCH].Kp, std::memory_order_relaxed);
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_GRIPPER].control_param.Kp.store(ctrl_buf[SHM_INDEX_RIGHT_GRIPPER].Kp, std::memory_order_relaxed);


        std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_PITCH].control_param.Kd.store(ctrl_buf[SHM_INDEX_LEFT_SHOULDER_PITCH].Kd, std::memory_order_relaxed);
        std::get<RS02_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_ROLL].control_param.Kd.store(ctrl_buf[SHM_INDEX_LEFT_SHOULDER_ROLL].Kd, std::memory_order_relaxed);
        std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_SHOULDER_YAW].control_param.Kd.store(ctrl_buf[SHM_INDEX_LEFT_SHOULDER_YAW].Kd, std::memory_order_relaxed);
        std::get<RS00_Vec>(AReuMii)[VEC_INDEX_LEFT_ELBOW_PITCH].control_param.Kd.store(ctrl_buf[SHM_INDEX_LEFT_ELBOW_PITCH].Kd, std::memory_order_relaxed);

        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_ROLL].control_param.Kd.store(ctrl_buf[SHM_INDEX_LEFT_WRIST_ROLL].Kd, std::memory_order_relaxed);
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_YAW].control_param.Kd.store(ctrl_buf[SHM_INDEX_LEFT_WRIST_YAW].Kd, std::memory_order_relaxed);
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_WRIST_PITCH].control_param.Kd.store(ctrl_buf[SHM_INDEX_LEFT_WRIST_PITCH].Kd, std::memory_order_relaxed);
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_LEFT_GRIPPER].control_param.Kd.store(ctrl_buf[SHM_INDEX_LEFT_GRIPPER].Kd, std::memory_order_relaxed);

        std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_PITCH].control_param.Kd.store(ctrl_buf[SHM_INDEX_RIGHT_SHOULDER_PITCH].Kd, std::memory_order_relaxed);
        std::get<RS02_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_ROLL].control_param.Kd.store(ctrl_buf[SHM_INDEX_RIGHT_SHOULDER_ROLL].Kd, std::memory_order_relaxed);
        std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_SHOULDER_YAW].control_param.Kd.store(ctrl_buf[SHM_INDEX_RIGHT_SHOULDER_YAW].Kd, std::memory_order_relaxed);
        std::get<RS00_Vec>(AReuMii)[VEC_INDEX_RIGHT_ELBOW_PITCH].control_param.Kd.store(ctrl_buf[SHM_INDEX_RIGHT_ELBOW_PITCH].Kd, std::memory_order_relaxed);

        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_ROLL].control_param.Kd.store(ctrl_buf[SHM_INDEX_RIGHT_WRIST_ROLL].Kd, std::memory_order_relaxed);
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_YAW].control_param.Kd.store(ctrl_buf[SHM_INDEX_RIGHT_WRIST_YAW].Kd, std::memory_order_relaxed);
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_WRIST_PITCH].control_param.Kd.store(ctrl_buf[SHM_INDEX_RIGHT_WRIST_PITCH].Kd, std::memory_order_relaxed);
        std::get<EL05_Vec>(AReuMii)[VEC_INDEX_RIGHT_GRIPPER].control_param.Kd.store(ctrl_buf[SHM_INDEX_RIGHT_GRIPPER].Kd, std::memory_order_relaxed);




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
            for (auto& motor : vec) motor.init_motor_MIT(10, 10);
        }(vecs));
    }, AReuMii);

    //�ш린�� �ㅻ젅�� �앹꽦
    pthread_t rt_t, print_t, shm_t;
    pthread_create(&rt_t,    NULL, CAN_Comm_thread,   &can_interface);
    pthread_create(&print_t, NULL, print_thread_func,  nullptr);  // 珥덈컲 �뺤씤�⑹씠�� �섏쨷�먮뒗 �덉벐�� �ㅻ젅�쒖엫. 
    pthread_create(&shm_t, NULL, update_Control_params,  nullptr);



    pthread_join(rt_t,    nullptr);
    pthread_join(print_t, nullptr);
    pthread_join(shm_t , nullptr);

    return 0;
}
