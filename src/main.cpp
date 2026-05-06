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
constexpr long CONTROL_PERIOD = 2'000'000;
constexpr int MaxID = 32;

constexpr int MOTOR_NUM = 12;

Motor_con Leg;


void signal_handler(int signum) { running = false; }

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
        }, Leg);
        std::cout << "---\n";
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
        while (!shm_ptr->try_read_ctrl(ctrl_buf));  // torn read면 재시도
                                                                                                                                    // COMMANDS
                                                                                                                                // POS 
        std::get<RS04_Vec>(Leg)[0].control_param.pos.store(std::clamp(ctrl_buf[0].pos, MIN_POS_LEFT_HIP_PITCH, MAX_POS_LEFT_HIP_PITCH), std::memory_order_relaxed);  // L HIP PITCH 
        std::get<RS04_Vec>(Leg)[1].control_param.pos.store(std::clamp(ctrl_buf[1].pos, MIN_POS_RIGHT_HIP_PITCH, MAX_POS_RIGHT_HIP_PITCH), std::memory_order_relaxed);  // R HIP PITCH   
        std::get<RS04_Vec>(Leg)[2].control_param.pos.store(std::clamp(ctrl_buf[2].pos, MIN_POS_LEFT_KNEE_PITCH, MAX_POS_LEFT_KNEE_PITCH), std::memory_order_relaxed);  // L KNEE PITCH
        std::get<RS04_Vec>(Leg)[3].control_param.pos.store(std::clamp(ctrl_buf[3].pos, MIN_POS_RIGHT_KNEE_PITCH, MAX_POS_RIGHT_KNEE_PITCH), std::memory_order_relaxed);  // R KNEE PITCH

        std::get<RS03_Vec>(Leg)[0].control_param.pos.store(std::clamp(ctrl_buf[4].pos, MIN_POS_LEFT_HIP_ROLL, MAX_POS_LEFT_HIP_ROLL), std::memory_order_relaxed);  // L HIP ROLL
        std::get<RS03_Vec>(Leg)[1].control_param.pos.store(std::clamp(ctrl_buf[5].pos, MIN_POS_RIGHT_HIP_ROLL, MAX_POS_RIGHT_HIP_ROLL), std::memory_order_relaxed);  // R HIP ROLL
        std::get<RS03_Vec>(Leg)[2].control_param.pos.store(std::clamp(ctrl_buf[6].pos, MIN_POS_LEFT_HIP_YAW, MAX_POS_LEFT_HIP_YAW), std::memory_order_relaxed);  // L HIP YAW
        std::get<RS03_Vec>(Leg)[3].control_param.pos.store(std::clamp(ctrl_buf[7].pos, MIN_POS_RIGHT_HIP_YAW, MAX_POS_RIGHT_HIP_YAW), std::memory_order_relaxed);  // R HIP YAW

        std::get<RS06_Vec>(Leg)[0].control_param.pos.store(std::clamp(ctrl_buf[8].pos, -2.0, 2.0), std::memory_order_relaxed);  // L ANKLE A 
        std::get<RS06_Vec>(Leg)[1].control_param.pos.store(std::clamp(ctrl_buf[9].pos, -2.0, 2.0), std::memory_order_relaxed);  // R ANKLE A 
        std::get<RS06_Vec>(Leg)[2].control_param.pos.store(std::clamp(ctrl_buf[10].pos, -2.0, 2.0), std::memory_order_relaxed);  // L ANKLE B 
        std::get<RS06_Vec>(Leg)[3].control_param.pos.store(std::clamp(ctrl_buf[11].pos, -2.0, 2.0), std::memory_order_relaxed);  // R ANKLE B

                                                                                                            // Kp
        std::get<RS04_Vec>(Leg)[0].control_param.Kp.store(ctrl_buf[0].Kp, std::memory_order_relaxed);       //
        std::get<RS04_Vec>(Leg)[1].control_param.Kp.store(ctrl_buf[1].Kp, std::memory_order_relaxed);  
        std::get<RS04_Vec>(Leg)[2].control_param.Kp.store(ctrl_buf[2].Kp, std::memory_order_relaxed);  
        std::get<RS04_Vec>(Leg)[3].control_param.Kp.store(ctrl_buf[3].Kp, std::memory_order_relaxed);  

        std::get<RS03_Vec>(Leg)[0].control_param.Kp.store(ctrl_buf[4].Kp, std::memory_order_relaxed);       
        std::get<RS03_Vec>(Leg)[1].control_param.Kp.store(ctrl_buf[5].Kp, std::memory_order_relaxed);  
        std::get<RS03_Vec>(Leg)[2].control_param.Kp.store(ctrl_buf[6].Kp, std::memory_order_relaxed);  
        std::get<RS03_Vec>(Leg)[3].control_param.Kp.store(ctrl_buf[7].Kp, std::memory_order_relaxed);  
        
        std::get<RS06_Vec>(Leg)[0].control_param.Kp.store(ctrl_buf[8].Kp, std::memory_order_relaxed);       
        std::get<RS06_Vec>(Leg)[1].control_param.Kp.store(ctrl_buf[9].Kp, std::memory_order_relaxed);  
        std::get<RS06_Vec>(Leg)[2].control_param.Kp.store(ctrl_buf[10].Kp, std::memory_order_relaxed);  
        std::get<RS06_Vec>(Leg)[3].control_param.Kp.store(ctrl_buf[11].Kp, std::memory_order_relaxed);  

                                                                                                            //Kd
        std::get<RS04_Vec>(Leg)[0].control_param.Kd.store(ctrl_buf[0].Kd, std::memory_order_relaxed);       //
        std::get<RS04_Vec>(Leg)[1].control_param.Kd.store(ctrl_buf[1].Kd, std::memory_order_relaxed);  
        std::get<RS04_Vec>(Leg)[2].control_param.Kd.store(ctrl_buf[2].Kd, std::memory_order_relaxed);  
        std::get<RS04_Vec>(Leg)[3].control_param.Kd.store(ctrl_buf[3].Kd, std::memory_order_relaxed);  

        std::get<RS03_Vec>(Leg)[0].control_param.Kd.store(ctrl_buf[4].Kd, std::memory_order_relaxed);       
        std::get<RS03_Vec>(Leg)[1].control_param.Kd.store(ctrl_buf[5].Kd, std::memory_order_relaxed);  
        std::get<RS03_Vec>(Leg)[2].control_param.Kd.store(ctrl_buf[6].Kd, std::memory_order_relaxed);  
        std::get<RS03_Vec>(Leg)[3].control_param.Kd.store(ctrl_buf[7].Kd, std::memory_order_relaxed);  
        
        std::get<RS06_Vec>(Leg)[0].control_param.Kd.store(ctrl_buf[8].Kd, std::memory_order_relaxed);       
        std::get<RS06_Vec>(Leg)[1].control_param.Kd.store(ctrl_buf[9].Kd, std::memory_order_relaxed);  
        std::get<RS06_Vec>(Leg)[2].control_param.Kd.store(ctrl_buf[10].Kd, std::memory_order_relaxed);  
        std::get<RS06_Vec>(Leg)[3].control_param.Kd.store(ctrl_buf[11].Kd, std::memory_order_relaxed);  
                                                                                                                                    // END COMMANDS

        
                                                                                                                                        // FEEDBACKS
        fb_buf[0].pos = std::get<RS04_Vec>(Leg)[0].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS04_Vec>(Leg)[0].pos_offset;
        fb_buf[1].pos = std::get<RS04_Vec>(Leg)[1].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS04_Vec>(Leg)[1].pos_offset;
        fb_buf[2].pos = std::get<RS04_Vec>(Leg)[2].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS04_Vec>(Leg)[2].pos_offset;
        fb_buf[3].pos = std::get<RS04_Vec>(Leg)[3].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS04_Vec>(Leg)[3].pos_offset;

        fb_buf[4].pos = std::get<RS03_Vec>(Leg)[0].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS03_Vec>(Leg)[0].pos_offset;
        fb_buf[5].pos = std::get<RS03_Vec>(Leg)[1].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS03_Vec>(Leg)[1].pos_offset;
        fb_buf[6].pos = std::get<RS03_Vec>(Leg)[2].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS03_Vec>(Leg)[2].pos_offset;
        fb_buf[7].pos = std::get<RS03_Vec>(Leg)[3].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS03_Vec>(Leg)[3].pos_offset;

        fb_buf[8].pos = std::get<RS06_Vec>(Leg)[0].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS06_Vec>(Leg)[0].pos_offset;
        fb_buf[9].pos = std::get<RS06_Vec>(Leg)[1].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS06_Vec>(Leg)[1].pos_offset;
        fb_buf[10].pos = std::get<RS06_Vec>(Leg)[2].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS06_Vec>(Leg)[2].pos_offset;
        fb_buf[11].pos = std::get<RS06_Vec>(Leg)[3].Feedback_param.pos.load(std::memory_order_relaxed)+std::get<RS06_Vec>(Leg)[3].pos_offset;


        fb_buf[0].vel = std::get<RS04_Vec>(Leg)[0].Feedback_param.vel.load(std::memory_order_relaxed);      //VEL
        fb_buf[1].vel = std::get<RS04_Vec>(Leg)[1].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[2].vel = std::get<RS04_Vec>(Leg)[2].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[3].vel = std::get<RS04_Vec>(Leg)[3].Feedback_param.vel.load(std::memory_order_relaxed);

        fb_buf[4].vel = std::get<RS03_Vec>(Leg)[0].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[5].vel = std::get<RS03_Vec>(Leg)[1].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[6].vel = std::get<RS03_Vec>(Leg)[2].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[7].vel = std::get<RS03_Vec>(Leg)[3].Feedback_param.vel.load(std::memory_order_relaxed);

        fb_buf[8].vel = std::get<RS06_Vec>(Leg)[0].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[9].vel = std::get<RS06_Vec>(Leg)[1].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[10].vel = std::get<RS06_Vec>(Leg)[2].Feedback_param.vel.load(std::memory_order_relaxed);
        fb_buf[11].vel = std::get<RS06_Vec>(Leg)[3].Feedback_param.vel.load(std::memory_order_relaxed);


        fb_buf[0].torque = std::get<RS04_Vec>(Leg)[0].Feedback_param.torque.load(std::memory_order_relaxed);  //TORQUE
        fb_buf[1].torque = std::get<RS04_Vec>(Leg)[1].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[2].torque = std::get<RS04_Vec>(Leg)[2].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[3].torque = std::get<RS04_Vec>(Leg)[3].Feedback_param.torque.load(std::memory_order_relaxed);

        fb_buf[4].torque = std::get<RS03_Vec>(Leg)[0].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[5].torque = std::get<RS03_Vec>(Leg)[1].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[6].torque = std::get<RS03_Vec>(Leg)[2].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[7].torque = std::get<RS03_Vec>(Leg)[3].Feedback_param.torque.load(std::memory_order_relaxed);

        fb_buf[8].torque = std::get<RS06_Vec>(Leg)[0].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[9].torque = std::get<RS06_Vec>(Leg)[1].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[10].torque = std::get<RS06_Vec>(Leg)[2].Feedback_param.torque.load(std::memory_order_relaxed);
        fb_buf[11].torque = std::get<RS06_Vec>(Leg)[3].Feedback_param.torque.load(std::memory_order_relaxed);

                                                                                                                // 온도 피드백은 실제 제어할땐 필요없을 것으로 예상됨. 
        fb_buf[0].temp = std::get<RS04_Vec>(Leg)[0].Feedback_param.temp.load(std::memory_order_relaxed);  //TEMP
        fb_buf[1].temp = std::get<RS04_Vec>(Leg)[1].Feedback_param.temp.load(std::memory_order_relaxed);
        fb_buf[2].temp = std::get<RS04_Vec>(Leg)[2].Feedback_param.temp.load(std::memory_order_relaxed);
        fb_buf[3].temp = std::get<RS04_Vec>(Leg)[3].Feedback_param.temp.load(std::memory_order_relaxed);

        fb_buf[4].temp = std::get<RS03_Vec>(Leg)[0].Feedback_param.temp.load(std::memory_order_relaxed);
        fb_buf[5].temp = std::get<RS03_Vec>(Leg)[1].Feedback_param.temp.load(std::memory_order_relaxed);
        fb_buf[6].temp = std::get<RS03_Vec>(Leg)[2].Feedback_param.temp.load(std::memory_order_relaxed);
        fb_buf[7].temp = std::get<RS03_Vec>(Leg)[3].Feedback_param.temp.load(std::memory_order_relaxed);

        fb_buf[8].temp = std::get<RS06_Vec>(Leg)[0].Feedback_param.temp.load(std::memory_order_relaxed);
        fb_buf[9].temp = std::get<RS06_Vec>(Leg)[1].Feedback_param.temp.load(std::memory_order_relaxed);
        fb_buf[10].temp = std::get<RS06_Vec>(Leg)[2].Feedback_param.temp.load(std::memory_order_relaxed);
        fb_buf[11].temp = std::get<RS06_Vec>(Leg)[3].Feedback_param.temp.load(std::memory_order_relaxed);

        shm_ptr->write_fb(fb_buf);
                                                                                                                                    // END FEEDBACKS
                                                                                                                //피드백은 뭐 각 객체별 clamp도 없어서 반복문을 쓰려면 쓸순 있을듯함 다만 연결에 있어서 불편할듯. 

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
            for (auto& motor : vec) motor.init_motor_MIT(1, 10);
        }(vecs));
    }, Leg);

    //여기서 스레드 생성
    pthread_t rt_t, print_t, shm_t;
    pthread_create(&rt_t,    NULL, CAN_Comm_thread,   &can_interface);
    pthread_create(&print_t, NULL, print_thread_func,  nullptr);  // 초반 확인용이라 나중에는 안쓰는 스레드임. 
    pthread_create(&shm_t, NULL, update_Control_params,  nullptr);



    pthread_join(rt_t,    nullptr);
    pthread_join(print_t, nullptr);
    pthread_join(shm_t , nullptr);

    return 0;
}
