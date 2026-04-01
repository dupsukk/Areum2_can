#include "RobstrideMotor.hpp"
#include "Rx_handler.hpp"
#include <signal.h>
#include <vector>
#include <iostream>

#include <fcntl.h>
#include <sys/mman.h>


std::vector<Motortype> LeftArm;

const char* CAN_INTERFACE_0 = "can0";
const char* CAN_INTERFACE_1 = "can1";

bool running = true; 

Motor_con Left_Leg ;


void signal_handler(int signum) { running = false; }

int main(){

    signal(SIGINT, signal_handler);

    int s1 = init_can(CAN_INTERFACE_0);
    if (s1 < 0) return -1;

    can_frame cf;

    std::get<RS04_Vec>(Left_Leg).emplace_back(s1,0x01);
    std::get<RS03_Vec>(Left_Leg).emplace_back(s1,0x02);
    std::get<RS03_Vec>(Left_Leg).emplace_back(s1,0x03);
    std::get<RS04_Vec>(Left_Leg).emplace_back(s1,0x04);
    std::get<RS06_Vec>(Left_Leg).emplace_back(s1,0x05);
    std::get<RS06_Vec>(Left_Leg).emplace_back(s1,0x06);

    std::apply([](auto&... vecs) {
        (..., [](auto& vec) {
            for (auto& motor : vec) motor.init_motor_MIT(1,1);
        }(vecs));
    }, Left_Leg);

    Rx_handler hRx(Left_Leg) ;

    

    while(running){

        //여기서는 뭐 읽어오기! 
        for(auto i : LeftArm){
            if(readframe(s1, &cf)) {
                auto [id, err,pos,vel,torq,temp] = hRx.parse_Rx_frame(&cf);
                std::cout << "id : " <<id <<"  err :" << (err==0) <<"  pos : " <<pos << "  vel : " <<vel << "  torque : " <<torq;
            }
        }

        for(auto &i : LeftArm) {
            std::visit([&](auto& motor) {
                motor.write_operation_frame(motor.control_param.pos, motor.control_param.Kp, motor.control_param.Kd);
            }, i);

        };  
    }


    return 0;
}