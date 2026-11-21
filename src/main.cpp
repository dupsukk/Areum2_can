// 자동 생성 -- 고치지 말 것.  출처: robot.yaml  (생성기: gen_robot.py)
// YAML sha256[:16] = 5660e2c54f5f5f22
#include "AReuMii_config.h"
#include "can_runtime.hpp"

int main() {
    signal(SIGINT, signal_handler);

    std::vector<int> can_interface;
    for (const char* name : CAN_INTERFACE_NAMES) {
        int s = init_can(name);
        if (s < 0) return -1;
        can_interface.push_back(s);
    }

    emplace_motors(can_interface);
    init_motors();
    usleep(10000);

    // 여기서 스레드 생성
    pthread_t rt_t, print_t, shm_t;
    pthread_create(&rt_t,    NULL, CAN_Comm_thread,    &can_interface);
    pthread_create(&print_t, NULL, print_thread_func,  nullptr);  // 초반 확인용이라 나중에는 안쓰는 스레드임.
    pthread_create(&shm_t,   NULL, update_Control_params, nullptr);

    pthread_join(rt_t,    nullptr);
    pthread_join(print_t, nullptr);
    pthread_join(shm_t,   nullptr);

    if (g_startup_fault) return -1;  // 시작 시 영점 편차 초과 (CAN_Comm_thread 가 설정)
    return 0;
}
