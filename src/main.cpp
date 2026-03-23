#include "RobstrideMotor.hpp"
#include <signal.h>
#include <vector>

#include <fcntl.h>
#include <sys/mman.h>

/*
RobstrideMotor LeftArm[7] = {RobstrideMotor(RS02, CAN_ID_LEFT_SHOULDER_PITCH ) ,RobstrideMotor(RS02,CAN_ID_LEFT_SHOULDER_ROLL) , 
                                RobstrideMotor(RS00,CAN_ID_LEFT_SHOULDER_YAW) ,RobstrideMotor(RS00,CAN_ID_LEFT_ELBOW) ,
                                RobstrideMotor(EL05,CAN_ID_LEFT_WRIST_ROLL) ,RobstrideMotor(EL05,CAN_ID_LEFT_WRIST_PITCH) ,RobstrideMotor(EL05,CAN_ID_LEFT_WRIST_YAW) };


*/

//using Motortype = std::variant<RobstrideMotor<RS00> , RobstrideMotor<RS01>,RobstrideMotor<RS02> ,RobstrideMotor<RS03> ,RobstrideMotor<RS04> ,RobstrideMotor<RS05>,RobstrideMotor<RS06>,RobstrideMotor<EL05>>;

std::vector<Motortype> LeftArm;

const char* CAN_INTERFACE_0 = "can0";
const char* CAN_INTERFACE_1 = "can1";

bool running = true; 




void signal_handler(int signum) { running = false; }

int main(){

    signal(SIGINT, signal_handler);

    int s1 = init_can(CAN_INTERFACE_0);
    if (s1 < 0) return -1;


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

    

    while(running){
        for(auto &i : LeftArm) {
            std::visit([&](auto& motor) {
                motor.write_operation_frame(motor.control_param.pos, motor.control_param.Kp, motor.control_param.Kd);
            }, i);

        };  
        //TODO : 현재는 속도 인자도 없고 내부에서 무조건 하나의 디폴트 모터를 따라감 
        //FIXED : 모터타입을 탬플릿으로 설정함 

        //여기서는 뭐 읽어오기! 
    }


    return 0;
}