#include "RobstrideMotor.hpp"
#include "Rx_handler.hpp"
#include <signal.h>
#include <vector>
#include <iostream>

#include <fcntl.h>
#include <sys/mman.h>


const char* CAN_INTERFACE_0 = "can0";
const char* CAN_INTERFACE_1 = "can1";
bool running = true; 
constexpr long CONTROL_PERIOD = 2000000;
constexpr int MaxID = 32;

Motor_con Left_Leg ;


void signal_handler(int signum) { running = false; }
RealTimeClock RTC;

int main(){

    signal(SIGINT, signal_handler);

    int s1 = init_can(CAN_INTERFACE_0);
    if (s1 < 0) return -1;

    can_frame cf;

    //std::get<RS04_Vec>(Left_Leg).emplace_back(s1,0x01);
    //std::get<RS03_Vec>(Left_Leg).emplace_back(s1,0x02);
    //std::get<RS03_Vec>(Left_Leg).emplace_back(s1,0x03);
    //std::get<RS04_Vec>(Left_Leg).emplace_back(s1,0x04);
    //std::get<RS06_Vec>(Left_Leg).emplace_back(s1,0x05);
    //std::get<RS06_Vec>(Left_Leg).emplace_back(s1,0x06);
    std::get<EL05_Vec>(Left_Leg).emplace_back(s1,0x01);

    std::apply([](auto&... vecs) {
        (..., [](auto& vec) {
            for (auto& motor : vec) motor.init_motor_MIT(1,1);
        }(vecs));
    }, Left_Leg);

    Rx_handler<MaxID> hRx(Left_Leg) ;

    sleep(1);
    while (readframe(s1, &cf));

    RTC.reset();

    while(running){
        RTC.wait_next(CONTROL_PERIOD);

        while (readframe(s1, &cf)) {
            auto  [id, err,p,v,t,tem] = hRx.parse_Rx_frame(&cf);
            if(err)[[unlikely]] {
                std::cerr<<id<<"," <<" errorcode: " <<err; // 여기에 모터를 멈충는 함수를 넣어야 하는데 
                return -1;
            }else{
                hRx.Write_Fb(id, p,v,t,tem);  
            }
        }
        
        std::apply([](auto&... vecs) {
            (..., [](auto& vec) {
                //for (auto& motor : vec) motor.write_updated_operation_frame();
                for (auto& motor : vec) motor.write_operation_frame(0,0,0);
            }(vecs));
        }, Left_Leg);

    }
    return 0;
}