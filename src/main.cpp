#include "RobstrideMotor.hpp"
#include "Rx_handler.hpp"
#include "Sharemem.hpp"
#include <signal.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <ctime>

#include <fcntl.h>
#include <pthread.h>


const char* CAN_INTERFACE_0 = "can0";
const char* CAN_INTERFACE_1 = "can1";
bool running = true;
constexpr long CONTROL_PERIOD = 2'000'000;
constexpr int MaxID = 32;

constexpr int MOTOR_NUM = 12;

Motor_con Leg;


void signal_handler(int signum) { 
    running = false;
    std::apply([](auto&... vecs) {
        (..., [](auto& vec) {
            for (auto& motor : vec) {
                motor.write_operation_frame(0, 0, 0);
                motor.control_param.pos = 0;
                motor.control_param.Kp = 0;
                motor.control_param.Kd = 0;
        }
        }(vecs));
    }, Leg);
}

void* csv_log_thread_func(void*) {
    std::vector<char> io_buf(1 << 20);
    std::string line;

    std::cout << "log command: log [duration_sec=10] [hz=500]\n";

    while (running) {
        // 1초 타임아웃으로 stdin을 폴링 — running 플래그 체크 가능
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(STDIN_FILENO, &fds);
        struct timeval tv = {1, 0};
        if (select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv) <= 0) continue;
        if (!std::getline(std::cin, line)) break;

        std::istringstream iss(line);
        std::string cmd;
        iss >> cmd;
        if (cmd != "log") continue;

        double duration_sec = 10.0;
        double hz = 500.0;
        iss >> duration_sec >> hz;
        hz = std::clamp(hz, 1.0, 1000.0);

        long period_ns = static_cast<long>(1e9 / hz);
        long n_rows    = static_cast<long>(duration_sec * hz);

        char fname[64];
        {
            time_t now = time(nullptr);
            struct tm* t = localtime(&now);
            strftime(fname, sizeof(fname), "motor_log_%Y%m%d_%H%M%S.csv", t);
        }

        std::ofstream csv(fname);
        csv.rdbuf()->pubsetbuf(io_buf.data(), io_buf.size());

        csv << "time_ms"
            << ",L_HIP_PITCH_pos,L_HIP_PITCH_ref,L_HIP_PITCH_vel,L_HIP_PITCH_torq"
            << ",R_HIP_PITCH_pos,R_HIP_PITCH_ref,R_HIP_PITCH_vel,R_HIP_PITCH_torq"
            << ",L_KNEE_PITCH_pos,L_KNEE_PITCH_ref,L_KNEE_PITCH_vel,L_KNEE_PITCH_torq"
            << ",R_KNEE_PITCH_pos,R_KNEE_PITCH_ref,R_KNEE_PITCH_vel,R_KNEE_PITCH_torq"
            << ",L_HIP_ROLL_pos,L_HIP_ROLL_ref,L_HIP_ROLL_vel,L_HIP_ROLL_torq"
            << ",R_HIP_ROLL_pos,R_HIP_ROLL_ref,R_HIP_ROLL_vel,R_HIP_ROLL_torq"
            << ",L_HIP_YAW_pos,L_HIP_YAW_ref,L_HIP_YAW_vel,L_HIP_YAW_torq"
            << ",R_HIP_YAW_pos,R_HIP_YAW_ref,R_HIP_YAW_vel,R_HIP_YAW_torq"
            << ",L_ANKLE_A_pos,L_ANKLE_A_ref,L_ANKLE_A_vel,L_ANKLE_A_torq"
            << ",R_ANKLE_A_pos,R_ANKLE_A_ref,R_ANKLE_A_vel,R_ANKLE_A_torq"
            << ",L_ANKLE_B_pos,L_ANKLE_B_ref,L_ANKLE_B_vel,L_ANKLE_B_torq"
            << ",R_ANKLE_B_pos,R_ANKLE_B_ref,R_ANKLE_B_vel,R_ANKLE_B_torq"
            << '\n';

        std::cout << "logging " << duration_sec << "s @ " << hz << "Hz -> " << fname << "\n";

        struct timespec t0;
        clock_gettime(CLOCK_MONOTONIC, &t0);

        RealTimeClock rtc;
        for (long i = 0; i < n_rows && running; ++i) {
            rtc.wait_next(period_ns);

            struct timespec now;
            clock_gettime(CLOCK_MONOTONIC, &now);
            double elapsed_ms = (now.tv_sec - t0.tv_sec) * 1000.0
                              + (now.tv_nsec - t0.tv_nsec) / 1e6;

            csv << elapsed_ms;

            for (const auto& m : std::get<RS04_Vec>(Leg)) {
                csv << ',' << (m.Feedback_param.pos.load(std::memory_order_relaxed) + m.pos_offset)
                    << ',' << m.control_param.pos.load(std::memory_order_relaxed)
                    << ',' << m.Feedback_param.vel.load(std::memory_order_relaxed)
                    << ',' << m.Feedback_param.torque.load(std::memory_order_relaxed);
            }
            for (const auto& m : std::get<RS03_Vec>(Leg)) {
                csv << ',' << (m.Feedback_param.pos.load(std::memory_order_relaxed) + m.pos_offset)
                    << ',' << m.control_param.pos.load(std::memory_order_relaxed)
                    << ',' << m.Feedback_param.vel.load(std::memory_order_relaxed)
                    << ',' << m.Feedback_param.torque.load(std::memory_order_relaxed);
            }
            for (const auto& m : std::get<RS06_Vec>(Leg)) {
                csv << ',' << (m.Feedback_param.pos.load(std::memory_order_relaxed) + m.pos_offset)
                    << ',' << m.control_param.pos.load(std::memory_order_relaxed)
                    << ',' << m.Feedback_param.vel.load(std::memory_order_relaxed)
                    << ',' << m.Feedback_param.torque.load(std::memory_order_relaxed);
            }

            csv << '\n';
        }

        std::cout << "done: " << fname << "\n";
    }

    return nullptr;
}

void* CAN_Comm_thread(void* arg) {  // TODO : 캘리브레이션을 위한 로직을 따로 뺄 것 그리고 뭐 캔 인터페이스 갯수 맞출 수 있게 탬플릿을 넣거나 
    struct sched_param param;
    param.sched_priority = 99;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);

    Rx_handler<MaxID> hRx(Leg);
    can_frame cf;
    //int s1 = *(int*)arg;
    auto& can_interface_vec = *static_cast<std::vector<int>*>(arg);

    for(int s : can_interface_vec){
        while (readframe(s, &cf));
    }


    std::apply([](auto&... vecs) {
        (..., [](auto& vec) {
            for (auto& motor : vec) motor.write_operation_frame(0, 0, 0);
        }(vecs));
    }, Leg);

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
    }, Leg);

    RTC.reset();
    while (running) {
        RTC.wait_next(CONTROL_PERIOD);

        for(int s : can_interface_vec){    // 이 부분이 성능상 문제가 될 잠재적인 가능성이 있는데 그럴 경우 파리미터 업데이트 스레드로 옮기면 됨. 
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
                //for (auto& motor : vec) motor.write_updated_operation_frame();
                for (auto& motor : vec) motor.write_updated_operation_frame();
            }(vecs));
        }, Leg);
    }

    return NULL;
}

void* update_Control_params(void* args){

    Control_Shm<MOTOR_NUM> ctrl_shm(13563267);
    auto* shm_ptr = ctrl_shm.get();

    Control_param ctrl_buf[MOTOR_NUM];
    Feedback_Param fb_buf[MOTOR_NUM];
    

    while(running){                                    // 이 루프 모듈화는 힘들기도 하고 했을때 오히려 의도가 불명확해보일 수 있음. 따라서 로봇이 바뀐다면 이거 정도는 뭐 합시다. 어려운거 아니자네 

                                                                                                                                        // FEEDBACKS
        fb_buf[SHM_MOTOR_INDEX_LEFT_HIP_PITCH].pos = std::get<RS04_Vec>(Leg)[0].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS04_Vec>(Leg)[0].pos_offset;
        fb_buf[SHM_MOTOR_INDEX_RIGHT_HIP_PITCH].pos = std::get<RS04_Vec>(Leg)[1].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS04_Vec>(Leg)[1].pos_offset;
        fb_buf[SHM_MOTOR_INDEX_LEFT_KNEE_PITCH].pos = std::get<RS04_Vec>(Leg)[2].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS04_Vec>(Leg)[2].pos_offset;
        fb_buf[SHM_MOTOR_INDEX_RIGHT_KNEE_PITCH].pos = std::get<RS04_Vec>(Leg)[3].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS04_Vec>(Leg)[3].pos_offset;

        fb_buf[SHM_MOTOR_INDEX_LEFT_HIP_ROLL].pos = std::get<RS03_Vec>(Leg)[0].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS03_Vec>(Leg)[0].pos_offset;
        fb_buf[SHM_MOTOR_INDEX_RIGHT_HIP_ROLL].pos = std::get<RS03_Vec>(Leg)[1].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS03_Vec>(Leg)[1].pos_offset;
        fb_buf[SHM_MOTOR_INDEX_LEFT_HIP_YAW].pos = std::get<RS03_Vec>(Leg)[2].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS03_Vec>(Leg)[2].pos_offset;
        fb_buf[SHM_MOTOR_INDEX_IGHT_HIP_YAW].pos = std::get<RS03_Vec>(Leg)[3].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS03_Vec>(Leg)[3].pos_offset;

        fb_buf[SHM_MOTOR_INDEX_LEFT_ANKLE_A].pos = std::get<RS06_Vec>(Leg)[0].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS06_Vec>(Leg)[0].pos_offset;
        fb_buf[SHM_MOTOR_INDEX_RIGHT_ANKLE_A].pos = std::get<RS06_Vec>(Leg)[1].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS06_Vec>(Leg)[1].pos_offset;
        fb_buf[SHM_MOTOR_INDEX_LEFT_ANKLE_B].pos = std::get<RS06_Vec>(Leg)[2].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS06_Vec>(Leg)[2].pos_offset;
        fb_buf[SHM_MOTOR_INDEX_RIGHT_ANKLE_B].pos = std::get<RS06_Vec>(Leg)[3].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS06_Vec>(Leg)[3].pos_offset;


        fb_buf[SHM_MOTOR_INDEX_LEFT_HIP_PITCH].vel = std::get<RS04_Vec>(Leg)[0].Feedback_param.vel.load(std::memory_order_relaxed);      //VEL
        fb_buf[SHM_MOTOR_INDEX_RIGHT_HIP_PITCH].vel = std::get<RS04_Vec>(Leg)[1].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[SHM_MOTOR_INDEX_LEFT_KNEE_PITCH].vel = std::get<RS04_Vec>(Leg)[2].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[SHM_MOTOR_INDEX_RIGHT_KNEE_PITCH].vel = std::get<RS04_Vec>(Leg)[3].Feedback_param.vel.load(std::memory_order_relaxed);

        fb_buf[SHM_MOTOR_INDEX_LEFT_HIP_ROLL].vel = std::get<RS03_Vec>(Leg)[0].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[SHM_MOTOR_INDEX_RIGHT_HIP_ROLL].vel = std::get<RS03_Vec>(Leg)[1].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[SHM_MOTOR_INDEX_LEFT_HIP_YAW].vel = std::get<RS03_Vec>(Leg)[2].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[SHM_MOTOR_INDEX_IGHT_HIP_YAW].vel = std::get<RS03_Vec>(Leg)[3].Feedback_param.vel.load(std::memory_order_relaxed);

        fb_buf[SHM_MOTOR_INDEX_LEFT_ANKLE_A].vel = std::get<RS06_Vec>(Leg)[0].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[SHM_MOTOR_INDEX_RIGHT_ANKLE_A].vel = std::get<RS06_Vec>(Leg)[1].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[SHM_MOTOR_INDEX_LEFT_ANKLE_B].vel = std::get<RS06_Vec>(Leg)[2].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[SHM_MOTOR_INDEX_RIGHT_ANKLE_B].vel = std::get<RS06_Vec>(Leg)[3].Feedback_param.vel.load(std::memory_order_relaxed);


        fb_buf[SHM_MOTOR_INDEX_LEFT_HIP_PITCH].torque = std::get<RS04_Vec>(Leg)[0].Feedback_param.torque.load(std::memory_order_relaxed);  //TORQUE
        fb_buf[SHM_MOTOR_INDEX_RIGHT_HIP_PITCH].torque = std::get<RS04_Vec>(Leg)[1].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[SHM_MOTOR_INDEX_LEFT_KNEE_PITCH].torque = std::get<RS04_Vec>(Leg)[2].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[SHM_MOTOR_INDEX_RIGHT_KNEE_PITCH].torque = std::get<RS04_Vec>(Leg)[3].Feedback_param.torque.load(std::memory_order_relaxed);

        fb_buf[SHM_MOTOR_INDEX_LEFT_HIP_ROLL].torque = std::get<RS03_Vec>(Leg)[0].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[SHM_MOTOR_INDEX_RIGHT_HIP_ROLL].torque = std::get<RS03_Vec>(Leg)[1].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[SHM_MOTOR_INDEX_LEFT_HIP_YAW].torque = std::get<RS03_Vec>(Leg)[2].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[SHM_MOTOR_INDEX_IGHT_HIP_YAW].torque = std::get<RS03_Vec>(Leg)[3].Feedback_param.torque.load(std::memory_order_relaxed);

        fb_buf[SHM_MOTOR_INDEX_LEFT_ANKLE_A].torque = std::get<RS06_Vec>(Leg)[0].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[SHM_MOTOR_INDEX_RIGHT_ANKLE_A].torque = std::get<RS06_Vec>(Leg)[1].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[SHM_MOTOR_INDEX_LEFT_ANKLE_B].torque = std::get<RS06_Vec>(Leg)[2].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[SHM_MOTOR_INDEX_RIGHT_ANKLE_B].torque = std::get<RS06_Vec>(Leg)[3].Feedback_param.torque.load(std::memory_order_relaxed);

                                                                                                                // 온도 피드백은 실제 제어할땐 필요없을 것으로 예상됨. 
        fb_buf[SHM_MOTOR_INDEX_LEFT_HIP_PITCH].temp = std::get<RS04_Vec>(Leg)[0].Feedback_param.temp.load(std::memory_order_relaxed);  //TEMP
        fb_buf[SHM_MOTOR_INDEX_RIGHT_HIP_PITCH].temp = std::get<RS04_Vec>(Leg)[1].Feedback_param.temp.load(std::memory_order_relaxed);
        fb_buf[SHM_MOTOR_INDEX_LEFT_KNEE_PITCH].temp = std::get<RS04_Vec>(Leg)[2].Feedback_param.temp.load(std::memory_order_relaxed);
        fb_buf[SHM_MOTOR_INDEX_RIGHT_KNEE_PITCH].temp = std::get<RS04_Vec>(Leg)[3].Feedback_param.temp.load(std::memory_order_relaxed);

        fb_buf[SHM_MOTOR_INDEX_LEFT_HIP_ROLL].temp = std::get<RS03_Vec>(Leg)[0].Feedback_param.temp.load(std::memory_order_relaxed);
        fb_buf[SHM_MOTOR_INDEX_RIGHT_HIP_ROLL].temp = std::get<RS03_Vec>(Leg)[1].Feedback_param.temp.load(std::memory_order_relaxed);
        fb_buf[SHM_MOTOR_INDEX_LEFT_HIP_YAW].temp = std::get<RS03_Vec>(Leg)[2].Feedback_param.temp.load(std::memory_order_relaxed);
        fb_buf[SHM_MOTOR_INDEX_IGHT_HIP_YAW].temp = std::get<RS03_Vec>(Leg)[3].Feedback_param.temp.load(std::memory_order_relaxed);

        fb_buf[SHM_MOTOR_INDEX_LEFT_ANKLE_A].temp = std::get<RS06_Vec>(Leg)[0].Feedback_param.temp.load(std::memory_order_relaxed);
        fb_buf[SHM_MOTOR_INDEX_RIGHT_ANKLE_A].temp = std::get<RS06_Vec>(Leg)[1].Feedback_param.temp.load(std::memory_order_relaxed);
        fb_buf[SHM_MOTOR_INDEX_LEFT_ANKLE_B].temp = std::get<RS06_Vec>(Leg)[2].Feedback_param.temp.load(std::memory_order_relaxed);
        fb_buf[SHM_MOTOR_INDEX_RIGHT_ANKLE_B].temp = std::get<RS06_Vec>(Leg)[3].Feedback_param.temp.load(std::memory_order_relaxed);

        shm_ptr->write_fb(fb_buf);
                                                                                                                                    // END FEEDBACKS
                                                                                                                //피드백은 뭐 각 객체별 clamp도 없어서 반복문을 쓰려면 쓸순 있을듯함 다만 연결에 있어서 불편할듯. 

        while (!shm_ptr->try_read_ctrl(ctrl_buf));  // torn read면 재시도
                                                                                                                                    // COMMANDS
                                                                                                                                // POS 

        double clamped_pos;

        clamped_pos = fb_buf[SHM_MOTOR_INDEX_LEFT_HIP_PITCH].pos < MIN_POS_LEFT_HIP_PITCH ? MIN_POS_LEFT_HIP_PITCH
                    : ctrl_buf[SHM_MOTOR_INDEX_LEFT_HIP_PITCH].pos > MAX_POS_LEFT_HIP_PITCH ? MAX_POS_LEFT_HIP_PITCH
                    : ctrl_buf[SHM_MOTOR_INDEX_LEFT_HIP_PITCH].pos;
        std::get<RS04_Vec>(Leg)[0].control_param.pos.store(clamped_pos, std::memory_order_relaxed);  // L HIP PITCH

        clamped_pos = fb_buf[SHM_MOTOR_INDEX_RIGHT_HIP_PITCH].pos < MIN_POS_RIGHT_HIP_PITCH ? MIN_POS_RIGHT_HIP_PITCH
                    : ctrl_buf[SHM_MOTOR_INDEX_RIGHT_HIP_PITCH].pos > MAX_POS_RIGHT_HIP_PITCH ? MAX_POS_RIGHT_HIP_PITCH
                    : ctrl_buf[SHM_MOTOR_INDEX_RIGHT_HIP_PITCH].pos;
        std::get<RS04_Vec>(Leg)[1].control_param.pos.store(clamped_pos, std::memory_order_relaxed);  // R HIP PITCH

        clamped_pos = fb_buf[SHM_MOTOR_INDEX_LEFT_KNEE_PITCH].pos < MIN_POS_LEFT_KNEE_PITCH ? MIN_POS_LEFT_KNEE_PITCH
                    : ctrl_buf[SHM_MOTOR_INDEX_LEFT_KNEE_PITCH].pos > MAX_POS_LEFT_KNEE_PITCH ? MAX_POS_LEFT_KNEE_PITCH
                    : ctrl_buf[SHM_MOTOR_INDEX_LEFT_KNEE_PITCH].pos;
        std::get<RS04_Vec>(Leg)[2].control_param.pos.store(clamped_pos, std::memory_order_relaxed);  // L KNEE PITCH

        clamped_pos = fb_buf[SHM_MOTOR_INDEX_RIGHT_KNEE_PITCH].pos < MIN_POS_RIGHT_KNEE_PITCH ? MIN_POS_RIGHT_KNEE_PITCH
                    : ctrl_buf[SHM_MOTOR_INDEX_RIGHT_KNEE_PITCH].pos > MAX_POS_RIGHT_KNEE_PITCH ? MAX_POS_RIGHT_KNEE_PITCH
                    : ctrl_buf[SHM_MOTOR_INDEX_RIGHT_KNEE_PITCH].pos;
        std::get<RS04_Vec>(Leg)[3].control_param.pos.store(clamped_pos, std::memory_order_relaxed);  // R KNEE PITCH

        clamped_pos = fb_buf[SHM_MOTOR_INDEX_LEFT_HIP_ROLL].pos < MIN_POS_LEFT_HIP_ROLL ? MIN_POS_LEFT_HIP_ROLL
                    : ctrl_buf[SHM_MOTOR_INDEX_LEFT_HIP_ROLL].pos > MAX_POS_LEFT_HIP_ROLL ? MAX_POS_LEFT_HIP_ROLL
                    : ctrl_buf[SHM_MOTOR_INDEX_LEFT_HIP_ROLL].pos;
        std::get<RS03_Vec>(Leg)[0].control_param.pos.store(clamped_pos, std::memory_order_relaxed);  // L HIP ROLL

        clamped_pos = fb_buf[SHM_MOTOR_INDEX_RIGHT_HIP_ROLL].pos < MIN_POS_RIGHT_HIP_ROLL ? MIN_POS_RIGHT_HIP_ROLL
                    : ctrl_buf[SHM_MOTOR_INDEX_RIGHT_HIP_ROLL].pos > MAX_POS_RIGHT_HIP_ROLL ? MAX_POS_RIGHT_HIP_ROLL
                    : ctrl_buf[SHM_MOTOR_INDEX_RIGHT_HIP_ROLL].pos;
        std::get<RS03_Vec>(Leg)[1].control_param.pos.store(clamped_pos, std::memory_order_relaxed);  // R HIP ROLL

        clamped_pos = fb_buf[SHM_MOTOR_INDEX_LEFT_HIP_YAW].pos < MIN_POS_LEFT_HIP_YAW ? MIN_POS_LEFT_HIP_YAW
                    : ctrl_buf[SHM_MOTOR_INDEX_LEFT_HIP_YAW].pos > MAX_POS_LEFT_HIP_YAW ? MAX_POS_LEFT_HIP_YAW
                    : ctrl_buf[SHM_MOTOR_INDEX_LEFT_HIP_YAW].pos;
        std::get<RS03_Vec>(Leg)[2].control_param.pos.store(clamped_pos, std::memory_order_relaxed);  // L HIP YAW

        clamped_pos = fb_buf[SHM_MOTOR_INDEX_IGHT_HIP_YAW].pos < MIN_POS_RIGHT_HIP_YAW ? MIN_POS_RIGHT_HIP_YAW
                    : ctrl_buf[SHM_MOTOR_INDEX_IGHT_HIP_YAW].pos > MAX_POS_RIGHT_HIP_YAW ? MAX_POS_RIGHT_HIP_YAW
                    : ctrl_buf[SHM_MOTOR_INDEX_IGHT_HIP_YAW].pos;
        std::get<RS03_Vec>(Leg)[3].control_param.pos.store(clamped_pos, std::memory_order_relaxed);  // R HIP YAW

        clamped_pos = fb_buf[SHM_MOTOR_INDEX_LEFT_ANKLE_A].pos < -2.0 ? -2.0
                    : ctrl_buf[SHM_MOTOR_INDEX_LEFT_ANKLE_A].pos > 2.0 ? 2.0
                    : ctrl_buf[SHM_MOTOR_INDEX_LEFT_ANKLE_A].pos;
        std::get<RS06_Vec>(Leg)[0].control_param.pos.store(clamped_pos, std::memory_order_relaxed);  // L ANKLE A

        clamped_pos = fb_buf[SHM_MOTOR_INDEX_RIGHT_ANKLE_A].pos < -2.0 ? -2.0
                    : ctrl_buf[SHM_MOTOR_INDEX_RIGHT_ANKLE_A].pos > 2.0 ? 2.0
                    : ctrl_buf[SHM_MOTOR_INDEX_RIGHT_ANKLE_A].pos;
        std::get<RS06_Vec>(Leg)[1].control_param.pos.store(clamped_pos, std::memory_order_relaxed);  // R ANKLE A

        clamped_pos = fb_buf[SHM_MOTOR_INDEX_LEFT_ANKLE_B].pos < -2.0 ? -2.0
                    : ctrl_buf[SHM_MOTOR_INDEX_LEFT_ANKLE_B].pos > 2.0 ? 2.0
                    : ctrl_buf[SHM_MOTOR_INDEX_LEFT_ANKLE_B].pos;
        std::get<RS06_Vec>(Leg)[2].control_param.pos.store(clamped_pos, std::memory_order_relaxed);  // L ANKLE B

        clamped_pos = fb_buf[SHM_MOTOR_INDEX_RIGHT_ANKLE_B].pos < -2.0 ? -2.0
                    : ctrl_buf[SHM_MOTOR_INDEX_RIGHT_ANKLE_B].pos > 2.0 ? 2.0
                    : ctrl_buf[SHM_MOTOR_INDEX_RIGHT_ANKLE_B].pos;
        std::get<RS06_Vec>(Leg)[3].control_param.pos.store(clamped_pos, std::memory_order_relaxed);  // R ANKLE B

                                                                                                            // Kp
        std::get<RS04_Vec>(Leg)[0].control_param.Kp.store(ctrl_buf[SHM_MOTOR_INDEX_LEFT_HIP_PITCH].Kp, std::memory_order_relaxed);       //
        std::get<RS04_Vec>(Leg)[1].control_param.Kp.store(ctrl_buf[SHM_MOTOR_INDEX_RIGHT_HIP_PITCH].Kp, std::memory_order_relaxed);  
        std::get<RS04_Vec>(Leg)[2].control_param.Kp.store(ctrl_buf[SHM_MOTOR_INDEX_LEFT_KNEE_PITCH].Kp, std::memory_order_relaxed);  
        std::get<RS04_Vec>(Leg)[3].control_param.Kp.store(ctrl_buf[SHM_MOTOR_INDEX_RIGHT_KNEE_PITCH].Kp, std::memory_order_relaxed);  

        std::get<RS03_Vec>(Leg)[0].control_param.Kp.store(ctrl_buf[SHM_MOTOR_INDEX_LEFT_HIP_ROLL].Kp, std::memory_order_relaxed);       
        std::get<RS03_Vec>(Leg)[1].control_param.Kp.store(ctrl_buf[SHM_MOTOR_INDEX_RIGHT_HIP_ROLL].Kp, std::memory_order_relaxed);  
        std::get<RS03_Vec>(Leg)[2].control_param.Kp.store(ctrl_buf[SHM_MOTOR_INDEX_LEFT_HIP_YAW].Kp, std::memory_order_relaxed);  
        std::get<RS03_Vec>(Leg)[3].control_param.Kp.store(ctrl_buf[SHM_MOTOR_INDEX_IGHT_HIP_YAW].Kp, std::memory_order_relaxed);  
        
        std::get<RS06_Vec>(Leg)[0].control_param.Kp.store(ctrl_buf[SHM_MOTOR_INDEX_LEFT_ANKLE_A].Kp, std::memory_order_relaxed);       
        std::get<RS06_Vec>(Leg)[1].control_param.Kp.store(ctrl_buf[SHM_MOTOR_INDEX_RIGHT_ANKLE_A].Kp, std::memory_order_relaxed);  
        std::get<RS06_Vec>(Leg)[2].control_param.Kp.store(ctrl_buf[SHM_MOTOR_INDEX_LEFT_ANKLE_B].Kp, std::memory_order_relaxed);  
        std::get<RS06_Vec>(Leg)[3].control_param.Kp.store(ctrl_buf[SHM_MOTOR_INDEX_RIGHT_ANKLE_B].Kp, std::memory_order_relaxed);  

                                                                                                            //Kd
        std::get<RS04_Vec>(Leg)[0].control_param.Kd.store(ctrl_buf[SHM_MOTOR_INDEX_LEFT_HIP_PITCH].Kd, std::memory_order_relaxed);       //
        std::get<RS04_Vec>(Leg)[1].control_param.Kd.store(ctrl_buf[SHM_MOTOR_INDEX_RIGHT_HIP_PITCH].Kd, std::memory_order_relaxed);  
        std::get<RS04_Vec>(Leg)[2].control_param.Kd.store(ctrl_buf[SHM_MOTOR_INDEX_LEFT_KNEE_PITCH].Kd, std::memory_order_relaxed);  
        std::get<RS04_Vec>(Leg)[3].control_param.Kd.store(ctrl_buf[SHM_MOTOR_INDEX_RIGHT_KNEE_PITCH].Kd, std::memory_order_relaxed);  

        std::get<RS03_Vec>(Leg)[0].control_param.Kd.store(ctrl_buf[SHM_MOTOR_INDEX_LEFT_HIP_ROLL].Kd, std::memory_order_relaxed);       
        std::get<RS03_Vec>(Leg)[1].control_param.Kd.store(ctrl_buf[SHM_MOTOR_INDEX_RIGHT_HIP_ROLL].Kd, std::memory_order_relaxed);  
        std::get<RS03_Vec>(Leg)[2].control_param.Kd.store(ctrl_buf[SHM_MOTOR_INDEX_LEFT_HIP_YAW].Kd, std::memory_order_relaxed);  
        std::get<RS03_Vec>(Leg)[3].control_param.Kd.store(ctrl_buf[SHM_MOTOR_INDEX_IGHT_HIP_YAW].Kd, std::memory_order_relaxed);  
        
        std::get<RS06_Vec>(Leg)[0].control_param.Kd.store(ctrl_buf[SHM_MOTOR_INDEX_LEFT_ANKLE_A].Kd, std::memory_order_relaxed);       
        std::get<RS06_Vec>(Leg)[1].control_param.Kd.store(ctrl_buf[SHM_MOTOR_INDEX_RIGHT_ANKLE_A].Kd, std::memory_order_relaxed);  
        std::get<RS06_Vec>(Leg)[2].control_param.Kd.store(ctrl_buf[SHM_MOTOR_INDEX_LEFT_ANKLE_B].Kd, std::memory_order_relaxed);  
        std::get<RS06_Vec>(Leg)[3].control_param.Kd.store(ctrl_buf[SHM_MOTOR_INDEX_RIGHT_ANKLE_B].Kd, std::memory_order_relaxed);  
                                                                                                                                    // END COMMANDS
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

    std::vector<int> can_interface = {s1, s2};

    /*
    std::get<RS02_Vec>(Leg).emplace_back(s1, 1);
    std::get<RS02_Vec>(Leg).emplace_back(s1, 2);
    std::get<RS00_Vec>(Leg).emplace_back(s1, 3);
    std::get<RS00_Vec>(Leg).emplace_back(s1, 4);
    */

   std::get<RS04_Vec>(Leg).emplace_back(s1, CAN_ID_LEFT_HIP_PITCH);
   std::get<RS04_Vec>(Leg).emplace_back(s2, CAN_ID_RIGHT_HIP_PITCH);

   std::get<RS03_Vec>(Leg).emplace_back(s1, CAN_ID_LEFT_HIP_ROLL);
   std::get<RS03_Vec>(Leg).emplace_back(s2, CAN_ID_RIGHT_HIP_ROLL);
   std::get<RS03_Vec>(Leg).emplace_back(s1, CAN_ID_LEFT_HIP_YAW);
   std::get<RS03_Vec>(Leg).emplace_back(s2, CAN_ID_RIGHT_HIP_YAW);

   std::get<RS04_Vec>(Leg).emplace_back(s1, CAN_ID_LEFT_KNEE_PITCH);
   std::get<RS04_Vec>(Leg).emplace_back(s2, CAN_ID_RIGHT_KNEE_PITCH);

   std::get<RS06_Vec>(Leg).emplace_back(s1, CAN_ID_LEFT_ANKLE_A);
   std::get<RS06_Vec>(Leg).emplace_back(s2, CAN_ID_RIGHT_ANKLE_A);
   std::get<RS06_Vec>(Leg).emplace_back(s1, CAN_ID_LEFT_ANKLE_B);
   std::get<RS06_Vec>(Leg).emplace_back(s2, CAN_ID_RIGHT_ANKLE_B);



    std::apply([](auto&... vecs) {
        (..., [](auto& vec) {
            for (auto& motor : vec) motor.init_motor_MIT(10, 20);
        }(vecs));
    }, Leg);

    //여기서 스레드 생성
    pthread_t rt_t, print_t, shm_t;
    pthread_create(&rt_t,    NULL, CAN_Comm_thread,   &can_interface);
    pthread_create(&print_t, NULL, csv_log_thread_func, nullptr);
    pthread_create(&shm_t, NULL, update_Control_params,  nullptr);



    pthread_join(rt_t,    nullptr);
    pthread_join(print_t, nullptr);
    pthread_join(shm_t , nullptr);

    return 0;
}
