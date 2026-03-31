#include "RobstrideMotor.hpp"
#include <signal.h>
#include <vector>

#include <fcntl.h>
#include <sys/mman.h>


std::vector<Motortype> LeftArm;

const char* CAN_INTERFACE_0 = "can0";
const char* CAN_INTERFACE_1 = "can1";

bool running = true; 




void signal_handler(int signum) { running = false; }

int main(){

    signal(SIGINT, signal_handler);

    int s1 = init_can(CAN_INTERFACE_0);
    if (s1 < 0) return -1;

    LeftArm.push_back(RobstrideMotor<RobstrideMotor_type::RS02>(s1 ,CAN_ID_LEFT_SHOULDER_PITCH));  // 이런 방식으로 확장쓰~ 
    can_frame cf;


    for(auto &i : LeftArm) {
        std::visit([&](auto& motor) {
            motor.enable_motor();
        }, i);
        usleep(10000); // 10ms delay
        std::visit([&](auto& motor) {
            motor.set_mode_raw(0);
        }, i);
        usleep(10000); // 10ms delay
        std::visit([&](auto& motor) {
            motor.write_limit(PARAM_VELOCITY_LIMIT, 5);
        }, i);
        usleep(10000); // 10ms delay
        std::visit([&](auto& motor) {
            motor.write_limit(PARAM_TORQUE_LIMIT, 5);
        }, i);
        usleep(10000); // 10ms delay
    };                                        // TODO: init 함수로 모드설정, 리밋 거는일 한번에 할 수 있도록 
                                            // init_motor_MIT 라는 함수로 구현함요 

    

    while(running){

        //여기서는 뭐 읽어오기! 
        for(auto i : LeftArm){
            readframe(s1, &cf);
                                         // 여기에다 
        }

        for(auto &i : LeftArm) {
            std::visit([&](auto& motor) {
                motor.write_operation_frame(motor.control_param.pos, motor.control_param.Kp, motor.control_param.Kd);
            }, i);

        };  

    }


    return 0;
}