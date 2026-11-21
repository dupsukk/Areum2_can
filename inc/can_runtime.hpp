#ifndef CAN_RUNTIME_HPP
#define CAN_RUNTIME_HPP

// 로봇에 무관한 고정 런타임 로직. 로봇별 표(액추에이터/부호/한계/인덱스)는 생성되는
// {robot_name}_config.h 와 main.cpp 에 있다. 이 헤더보다 먼저 config 헤더를 include 할 것.
#ifndef ROBOT_CONFIG_INCLUDED
#error "can_runtime.hpp 보다 먼저 생성된 {robot_name}_config.h 를 include 해야 합니다"
#endif

#include "RobstrideMotor.hpp"
#include "Rx_handler.hpp"
#include "Sharemem.hpp"
#include "utils.hpp"

#include <signal.h>
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#include <algorithm>
#include <iostream>
#include <tuple>
#include <vector>

inline Motor_con g_motors;
inline bool g_running = true;

// 모든 타입 deque 의 모든 모터에 f(motor) 를 적용한다.
template <class F>
inline void for_each_motor(F&& f) {
    std::apply([&](auto&... vecs) {
        (..., [&](auto& vec) {
            for (auto& motor : vec) f(motor);
        }(vecs));
    }, g_motors);
}

inline void signal_handler(int) {
    g_running = false;
    for_each_motor([](auto& motor) {
        motor.write_operation_frame(0, 0, 0);
        motor.control_param.pos = 0;
        motor.control_param.Kp = 0;
        motor.control_param.Kd = 0;
        motor.stop_motor();
    });
}

inline void* print_thread_func(void*) {
    while (g_running) {
        usleep(500000);
        for_each_motor([](const auto& motor) {
            std::cout
                << "[ID 0x" << std::hex << motor.can_id << std::dec << "] "
                << "pos="   << motor.Feedback_param.pos.load(std::memory_order_relaxed)
                << " corr=" << motor.Feedback_param.pos.load(std::memory_order_relaxed) + motor.pos_offset
                << " vel="  << motor.Feedback_param.vel.load(std::memory_order_relaxed)
                << " torq=" << motor.Feedback_param.torque.load(std::memory_order_relaxed)
                << " temp=" << motor.Feedback_param.temp.load(std::memory_order_relaxed)
                << " off="  << motor.pos_offset
                << "\n";
        });
        std::cout << "---\n";
    }
    return nullptr;
}

// arg: std::vector<int>* (열린 CAN 소켓들)
inline void* CAN_Comm_thread(void* arg) {  // TODO : 캘리브레이션을 위한 로직을 따로 뺄 것
    struct sched_param param;
    param.sched_priority = 99;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);

    Rx_handler<CAN_MAX_ID> hRx(g_motors);
    can_frame cf;
    auto& can_interface_vec = *static_cast<std::vector<int>*>(arg);

    for (int s : can_interface_vec) {
        while (readframe(s, &cf));
    }

    for_each_motor([](auto& motor) { motor.write_operation_frame(0, 0, 0); });

    // 첫 사이클: 피드백 수신 후 모든 모터 offset 캘리브레이션
    RealTimeClock RTC;
    RTC.wait_next(CONTROL_PERIOD_NS);

    for (int s : can_interface_vec) {
        while (readframe(s, &cf)) {
            auto [id, err, p, v, t, tem] = hRx.parse_Rx_frame(&cf);
            if (!err) hRx.Write_Fb(id, p, v, t, tem);
        }
    }

    for_each_motor([](auto& motor) { motor.calibrate(); });

    RTC.reset();
    while (g_running) {
        RTC.wait_next(CONTROL_PERIOD_NS);

        for (int s : can_interface_vec) {
            while (readframe(s, &cf)) {
                auto [id, err, p, v, t, tem] = hRx.parse_Rx_frame(&cf);
                if (err) [[unlikely]] {
                    std::cerr << id << " errorcode: " << err;
                    return nullptr;
                }
                hRx.Write_Fb(id, p, v, t, tem);
            }
        }

        // 모든 모터 동일 처리 (그리퍼 특수 취급 없음)
        for_each_motor([](auto& motor) { motor.write_updated_operation_frame(); });
    }

    return nullptr;
}

// ---- shm <-> 모터 변환 (좌표계: shm = 관절 좌표계, 모터 = sign 곱하기 전) ----

// 한계 안에서는 명령을 그대로 통과시키고, 피드백이 한계 밖일 때만 바깥쪽 명령을 막는다.
// (fb, lo, hi 는 모두 shm 좌표계)
inline double limit_pos_cmd(double cmd, double fb, double lo, double hi) {
    if (fb < lo) return std::max(cmd, lo);
    if (fb > hi) return std::min(cmd, hi);
    return cmd;
}

// 모터 피드백 -> shm 피드백. pos/vel/torque 모두 sign 적용, pos 에만 오프셋 적용.
template <class Motor>
inline void read_feedback(const Motor& m, Feedback_Param& out, double sign) {
    out.pos    = static_cast<float>(sign * (m.Feedback_param.pos.load(std::memory_order_relaxed) + m.pos_offset));
    out.vel    = static_cast<float>(sign * m.Feedback_param.vel.load(std::memory_order_relaxed));
    out.torque = static_cast<float>(sign * m.Feedback_param.torque.load(std::memory_order_relaxed));
}

// shm 명령 -> 모터 명령. fb_pos 는 같은 액추에이터의 shm 좌표계 피드백 위치.
// ffTorque 는 기존 동작대로 전달하지 않는다.
template <class Motor>
inline void write_command(Motor& m, const Control_param& c, double fb_pos,
                          double sign, double min_pos, double max_pos) {
    const double pos = limit_pos_cmd(c.pos, fb_pos, min_pos, max_pos);
    m.control_param.pos.store(sign * pos,   std::memory_order_relaxed);
    m.control_param.vel.store(sign * c.vel, std::memory_order_relaxed);
    m.control_param.Kp.store(c.Kp,          std::memory_order_relaxed);
    m.control_param.Kd.store(c.Kd,          std::memory_order_relaxed);
}

// 모터 객체 생성만 한다 (CAN 전송 없음). shm 인덱스 순서로 돈다 (VEC_INDEX 가 이 순서로 계산됨).
// 오프셋은 sign * offset 으로 모터에 전달한다. emplace_motor / 표는 생성된 config 헤더에 있다.
// 모든 모터가 만들어진 뒤에 init_motors_MIT 를 호출할 것.
inline void emplace_motors(const std::vector<int>& can_interface) {
    for (int i = 0; i < MOTOR_NUM; ++i) {
        emplace_motor(g_motors, i, can_interface[SHM_CAN_IF_INDEX[i]],
                      static_cast<int>(shm_index_to_can_id(i)), SHM_SIGN[i] * SHM_MECH_OFFSET[i]);
    }
}

// 모든 모터를 MIT 모드로 초기화한다 (CAN 전송 발생). emplace_motors 이후에 호출.
// 모터 문제로 한 번만 init 하면 안 되어서, 실전과 같이 두 번 초기화한다:
//   1) init -> stop (모터마다 10ms 간격)   2) 100ms 대기   3) init 다시
inline void init_motors_MIT() {
    for (int i = 0; i < MOTOR_NUM; ++i) {
        with_motor(g_motors, i, [&](auto& m) {
            m.init_motor_MIT(SHM_VEL_LIMIT[i], SHM_TORQUE_LIMIT[i]);
            m.stop_motor();
            usleep(10000);
        });
    }

    usleep(100000);

    for (int i = 0; i < MOTOR_NUM; ++i) {
        with_motor(g_motors, i, [&](auto& m) { m.init_motor_MIT(SHM_VEL_LIMIT[i], SHM_TORQUE_LIMIT[i]); });
    }
}

// shm <-> 모터 동기화 스레드.
// 순서: 피드백(모터 -> shm) 먼저, 그 다음 명령(shm -> 모터). 한계 클램프가 이번 주기의 피드백을 쓰기 때문.
// 인덱스 i 는 shm 인덱스. 표(SHM_*)와 with_motor 는 생성된 config 헤더에 있다.
inline void* update_Control_params(void*) {
    Control_Shm<MOTOR_NUM> ctrl_shm(ROBOT_SHM_KEY);
    auto* shm_ptr = ctrl_shm.get();

    Control_param ctrl_buf[MOTOR_NUM];
    Feedback_Param fb_buf[MOTOR_NUM];

    shm_ptr->write_ctrl(ctrl_buf);

    while (g_running) {
        while (!shm_ptr->try_read_ctrl(ctrl_buf));  // torn read 면 재시도

        for (int i = 0; i < MOTOR_NUM; ++i) {
            with_motor(g_motors, i, [&](auto& m) { read_feedback(m, fb_buf[i], SHM_SIGN[i]); });
        }
        for (int i = 0; i < MOTOR_NUM; ++i) {
            with_motor(g_motors, i, [&](auto& m) {
                write_command(m, ctrl_buf[i], fb_buf[i].pos, SHM_SIGN[i], SHM_MIN_POS[i], SHM_MAX_POS[i]);
            });
        }

        shm_ptr->write_fb(fb_buf);

        usleep(SHM_SYNC_PERIOD_US);
    }
    return nullptr;
}

#endif  // CAN_RUNTIME_HPP
