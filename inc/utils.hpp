#pragma once
#include <time.h>
#include <stdint.h>

class RealTimeClock {
    private:
    struct timespec next_time_;

    public:

    RealTimeClock(){
        clock_gettime(CLOCK_MONOTONIC, &next_time_);
    }
    
    void wait_next(long nano_sleeptime) {
        next_time_.tv_nsec += nano_sleeptime;
        if (next_time_.tv_nsec >= 1'000'000'000) {
            next_time_.tv_sec += 1;
            next_time_.tv_nsec -= 1'000'000'000;
        }

        // 한 주기 이상 늦었으면 따라잡지 않고 기준 시각을 현재로 다시 잡는다.
        // (그대로 두면 목표 시각이 과거라 사이클이 연달아 즉시 실행되어 CAN 프레임이 버스트로 나간다.)
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        const long late_ns = (now.tv_sec - next_time_.tv_sec) * 1'000'000'000L
                           + (now.tv_nsec - next_time_.tv_nsec);
        if (late_ns >= nano_sleeptime) next_time_ = now;

        while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time_, nullptr) != 0) ;
    }

    void reset() {
        clock_gettime(CLOCK_MONOTONIC, &next_time_);
    }

};