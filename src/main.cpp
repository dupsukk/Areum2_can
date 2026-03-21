#include "RobstrideMotor.hpp"
#include <signal.h>

#include <fcntl.h>
#include <sys/mman.h>


RobstrideMotor LeftArm[7] = {RobstrideMotor(RS02, CAN_ID_LEFT_SHOULDER_PITCH ) ,RobstrideMotor(RS02,CAN_ID_LEFT_SHOULDER_ROLL) , 
                                RobstrideMotor(RS00,CAN_ID_LEFT_SHOULDER_YAW) ,RobstrideMotor(RS00,CAN_ID_LEFT_ELBOW) ,
                                RobstrideMotor(EL05,CAN_ID_LEFT_WRIST_ROLL) ,RobstrideMotor(EL05,CAN_ID_LEFT_WRIST_PITCH) ,RobstrideMotor(EL05,CAN_ID_LEFT_WRIST_YAW) };

const char* CAN_INTERFACE_0 = "can0";
const char* CAN_INTERFACE_1 = "can1";

bool running = true; 




void signal_handler(int signum) { running = false; }

int main(){

    signal(SIGINT, signal_handler);

    int s1 = init_can(CAN_INTERFACE_0);
    if (s1 < 0) return -1;

    

    while(running){
        for(auto &i : LeftArm) i.write_operation_frame(s1,1,i.control_param.pos,i.control_param.Kp ,i.control_param.Kd);  
        //TODO : 현재는 속도 인자도 없고 내부에서 무조건 하나의 디폴트 모터를 따라감 

        //여기서는 뭐 읽어오기! 
    }


    return 0;
}