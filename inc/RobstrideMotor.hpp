#ifndef ROBSTRIDEMOTOR_HPP
#define ROBSTRIDEMOTOR_HPP

#include <stdint.h>
#include <stdlib.h>
#include <string>
#include <cstring>
#include <atomic>  // not using at this moment 
#include <stdio.h>
#include <unistd.h>
#include <cmath>
#include <variant>
#include <tuple>
#include <vector>
#include <algorithm>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "Areum2.hpp"
#include "utils.hpp"

const uint32_t COMM_WRITE_PARAMETER = 18;
const uint32_t COMM_ENABLE = 3;
const int HOST_ID = 0xFF;
const uint16_t PARAM_MODE = 0x7005;
const uint32_t COMM_OPERATION_CONTROL = 1;
const uint16_t PARAM_VELOCITY_LIMIT = 0x7017;
const uint16_t PARAM_TORQUE_LIMIT = 0x700B;           

enum class RobstrideMotor_type {RS00 = 0, RS01 =1 , RS02 =2 , RS03 = 3, RS04= 4, RS05 =5, RS06=6, EL05 = 7};

template <RobstrideMotor_type T> struct MotorConstants{
    static constexpr double POS_SCALE = 4 * M_PI;
    static constexpr double VEL_SCALE = 50.0;
    static constexpr double KP_SCALE = 500.0;
    static constexpr double KD_SCALE = 5.0;
    static constexpr double TQ_SCALE = 17.0;
};

template <> struct MotorConstants<RobstrideMotor_type::RS00>
{
    static constexpr double POS_SCALE = 4 * M_PI; // rs-00
    static constexpr double VEL_SCALE = 33.0;     // rs-00  
    static constexpr double KP_SCALE = 500.0;    // rs-00
    static constexpr double KD_SCALE = 5.0;     // rs-00
    static constexpr double TQ_SCALE = 14.0;      // rs-00  // 시트에는 이거 14 라는데 
};

template <> struct MotorConstants<RobstrideMotor_type::RS01>
{
    static constexpr double POS_SCALE = 4 * M_PI; // rs-01
    static constexpr double VEL_SCALE = 44.0;     // rs-01  // 시트에는 이거 44 라는데 
    static constexpr double KP_SCALE = 500.0;    // rs-01
    static constexpr double KD_SCALE = 5.0;     // rs-01
    static constexpr double TQ_SCALE = 17.0;      // rs-01
};

template <> struct MotorConstants<RobstrideMotor_type::RS02>
{
    static constexpr double POS_SCALE = 4 * M_PI; // rs-02
    static constexpr double VEL_SCALE = 44.0;     // rs-02
    static constexpr double KP_SCALE = 500.0;    // rs-02
    static constexpr double KD_SCALE = 5.0;     // rs-02
    static constexpr double TQ_SCALE = 17.0;      // rs-02    /* data */  이상 무 
};

template <> struct MotorConstants<RobstrideMotor_type::RS03>
{
    static constexpr double POS_SCALE = 4 * M_PI; // rs-03
    static constexpr double VEL_SCALE = 20.0;     // rs-03  // 시트에는 이거 20 이라는데 
    static constexpr double KP_SCALE = 5000.0;    // rs-03
    static constexpr double KD_SCALE = 100.0;     // rs-03
    static constexpr double TQ_SCALE = 60.0;      // rs-03    /* data */
};

template <> struct MotorConstants<RobstrideMotor_type::RS04>
{
    static constexpr double POS_SCALE = 4 * M_PI; // rs-04
    static constexpr double VEL_SCALE = 15.0;     // rs-04
    static constexpr double KP_SCALE = 5000.0;    // rs-04
    static constexpr double KD_SCALE = 100.0;     // rs-04
    static constexpr double TQ_SCALE = 120.0;      // rs-04   /* data */  이상 무 
};

template <> struct MotorConstants<RobstrideMotor_type::RS05>
{
    static constexpr double POS_SCALE = 4 * M_PI; // rs-05
    static constexpr double VEL_SCALE = 50.0;     // rs-05  // 시트에는 이거 50 이라는데 
    static constexpr double KP_SCALE = 500.0;    // rs-05
    static constexpr double KD_SCALE = 5.0;     // rs-05  
    static constexpr double TQ_SCALE = 5.5;      // rs-05    /* data */  // 시트에는 이거 5.5 라는데 
};

template <> struct MotorConstants<RobstrideMotor_type::RS06>
{
    static constexpr double POS_SCALE = 4 * M_PI; // rs-06
    static constexpr double VEL_SCALE = 50.0;     // rs-06// 시트에는 이거 50 이라는데 
    static constexpr double KP_SCALE = 5000.0;    // rs-06
    static constexpr double KD_SCALE = 100.0;     // rs-06      
    static constexpr double TQ_SCALE = 36.0;      // rs-06    /* data */  // 시트에는 이거 36 이라는데 
};

template <> struct MotorConstants<RobstrideMotor_type::EL05>  //EL05 데이터 확인 필요 
{
    static constexpr double POS_SCALE = 4 * M_PI; 
    static constexpr double VEL_SCALE = 50.0;     // 시트에는 이거 50 이라는데 
    static constexpr double KP_SCALE = 500.0;    
    static constexpr double KD_SCALE = 5.0;     
    static constexpr double TQ_SCALE = 6.0;         // 시트에는 이거 6 이라는데 
};

typedef struct 
{
    double pos;
    double vel;
    double Kp;
    double Kd;
    double ffTorque;
}Control_param;

typedef struct{          // TODO : 프로토콜에 맞춰서 피드백 데이터 넣기 아이디 확인해서 여기에 채워넣는거라 모터 말고 다른 클래스를 만드는게 좋음 
    float pos; 
    float vel;
    float torque;
    float temp;
}Feedback_Param;


template <RobstrideMotor_type Motor_type>
class RobstrideMotor {
    private :
    //const RobstrideMotor_type Motor_type;
    const int can_interface;


    public:
    const int can_id;
    Control_param control_param;
    Feedback_Param Feedback_param;


    RobstrideMotor(int can_interface,int can_id ): 
        can_interface(can_interface) ,can_id(can_id) {
            Feedback_param ={0};
    
    }
    
    
    void pack_float_le(uint8_t* buf, float val) {
        memcpy(buf, &val, sizeof(float));
    }
    
    // Copy uint16_t to uint8_t array (little-endian)
    void pack_u16_le(uint8_t* buf, uint16_t val) {
        memcpy(buf, &val, sizeof(uint16_t));
    }
    
    // Pack uint16_t in big-endian byte order
    void pack_u16_be(uint8_t* buf, uint16_t val) {
        buf[0] = (val >> 8) & 0xFF;
        buf[1] = val & 0xFF;
    }
    
    // --- Low-level CAN functions ---
    
    /**
     * @brief Send a CAN frame
     */
    bool send_frame( uint32_t can_id, const uint8_t* data, uint8_t dlc) {
        struct can_frame frame;
        frame.can_id = can_id | CAN_EFF_FLAG; // Enable extended frame (29-bit)
        frame.can_dlc = dlc;
        if (data) {
            memcpy(frame.data, data, dlc);
        } else {
            memset(frame.data, 0, 8);
        }
    
        if (write(can_interface, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
            perror("write");
            return false;
        }
        return true;
    }
    
    /**
     * @brief Read a CAN frame (with timeout)
     */
    bool read_frame(int s, struct can_frame* frame) {             
        // Set 100ms timeout
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 100000; // 100ms
        fd_set rdfs;
        FD_ZERO(&rdfs);
        FD_SET(s, &rdfs);
    
        int ret = select(s + 1, &rdfs, NULL, NULL, &tv);
        if (ret == -1) {
            perror("select");
            return false;
        } else if (ret == 0) {
            // std::cerr << "read timeout" << std::endl;
            return false; // Timeout
        }
    
        if (read(s, frame, sizeof(struct can_frame)) < 0) {
            perror("read");
            return false;
        }
        return true;
    }
    
    // --- RobStride protocol functions ---
    
    bool enable_motor() {
        uint32_t ext_id = (COMM_ENABLE << 24) | (HOST_ID << 8) | can_id;
        return send_frame( ext_id, nullptr, 0);
    }
    
    bool set_mode_raw(int8_t mode) {
        uint32_t ext_id = (COMM_WRITE_PARAMETER << 24) | (HOST_ID << 8) | can_id;
        uint8_t data[8] = {0};
        pack_u16_le(&data[0], PARAM_MODE); // param ID
        data[4] = (uint8_t)mode;           // value (int8)
        return send_frame(ext_id, data, 8);
    }
    
    bool write_limit(uint16_t param_id, float limit) {
        uint32_t ext_id = (COMM_WRITE_PARAMETER << 24) | (HOST_ID << 8) | can_id;
        uint8_t data[8] = {0};
        pack_u16_le(&data[0], param_id); // param ID
        pack_float_le(&data[4], limit);  // value (float)
        return send_frame(ext_id, data, 8);
    }
    
    bool write_operation_frame(double pos, double kp_val, double kd_val) {                     // TODO : 속도명령 넣을수 있도록. 또한 피드포워드 토크도 넣을 수 있도록. 
        // 1. Pack data (big-endian!)

                    // Clamp and convert
        double pos_clamped = std::max(-MotorConstants<Motor_type>::POS_SCALE, std::min(MotorConstants<Motor_type>::POS_SCALE, pos));
        double kp_clamped = std::max(0.0, std::min(MotorConstants<Motor_type>::KP_SCALE, kp_val));
        double kd_clamped = std::max(0.0, std::min(MotorConstants<Motor_type>::KD_SCALE, kd_val));                // std::clamp로 바꾸고 싶다 그죠? 
        
        uint16_t pos_u16 = (uint16_t)(((pos_clamped / MotorConstants<Motor_type>::POS_SCALE) + 1.0) * 0x7FFF);
        uint16_t vel_u16 = 0x7FFF; // 0 velocity
        uint16_t kp_u16 = (uint16_t)((kp_clamped / MotorConstants<Motor_type>::KP_SCALE) * 0xFFFF);
        uint16_t kd_u16 = (uint16_t)((kd_clamped / MotorConstants<Motor_type>::KD_SCALE) * 0xFFFF);         // 이거 나누기가 아니었는데? << 모터 상수 컨테이너에 역수를 추가하자 
        uint16_t torque_u16 = 0x7FFF; // 0 torque_ff
    
        uint8_t data[8];
        pack_u16_be(&data[0], pos_u16);
        pack_u16_be(&data[2], vel_u16);
        pack_u16_be(&data[4], kp_u16);
        pack_u16_be(&data[6], kd_u16);
        
            // 2. Construct CAN ID
            //uint32_t ext_id = (COMM_OPERATION_CONTROL << 24) | (torque_u16 << 8) | motor_id;      
        uint32_t ext_id = (COMM_OPERATION_CONTROL << 24) | ((0xffff&torque_u16) << 8) | can_id; // 정수형으로 변환되어 연산되는 경우가 잦으므로 비트마스킹을 하는것이 좋은 습관 
        
            // 3. Send
        return send_frame(ext_id, data, 8);
    
    }

    bool write_pos_pd_frame(double pos, double kp_val, double kd_val) {     // 이름처럼 포즈에 pd. 원래의 오퍼레이션 함수 원형에서 상수만 바꿈

                    // Clamp and convert
        double pos_clamped = std::max(-MotorConstants<Motor_type>::POS_SCALE, std::min(MotorConstants<Motor_type>::POS_SCALE, pos));
        double kp_clamped = std::max(0.0, std::min(MotorConstants<Motor_type>::KP_SCALE, kp_val));
        double kd_clamped = std::max(0.0, std::min(MotorConstants<Motor_type>::KD_SCALE, kd_val));               
        
        uint16_t pos_u16 = (uint16_t)(((pos_clamped / MotorConstants<Motor_type>::POS_SCALE) + 1.0) * 0x7FFF);
        uint16_t vel_u16 = 0x7FFF; // 0 velocity
        uint16_t kp_u16 = (uint16_t)((kp_clamped / MotorConstants<Motor_type>::KP_SCALE) * 0xFFFF);
        uint16_t kd_u16 = (uint16_t)((kd_clamped / MotorConstants<Motor_type>::KD_SCALE) * 0xFFFF);
        uint16_t torque_u16 = 0x7FFF; // 0 torque_ff
    
        uint8_t data[8];
        pack_u16_be(&data[0], pos_u16);
        pack_u16_be(&data[2], vel_u16);
        pack_u16_be(&data[4], kp_u16);
        pack_u16_be(&data[6], kd_u16);
        
        uint32_t ext_id = (COMM_OPERATION_CONTROL << 24) | ((0xffff&torque_u16) << 8) | can_id; 
        
        return send_frame(ext_id, data, 8);
    }

    bool write_updated_operation_frame() {  // 이건 가진 구조체에 뭐 다른 스레드건 shm을 올리건 해서 업데이트 된 정보를 쏘는 함수

        double pos_clamped = std::max(-MotorConstants<Motor_type>::POS_SCALE, std::min(MotorConstants<Motor_type>::POS_SCALE, control_param.pos));
        double kp_clamped = std::max(0.0, std::min(MotorConstants<Motor_type>::KP_SCALE, control_param.Kp));
        double kd_clamped = std::max(0.0, std::min(MotorConstants<Motor_type>::KD_SCALE, control_param.Kd));                // std::clamp로 바꾸고 싶다 그죠? 
        double vel_clamped = std::clamp(control_param.vel , -MotorConstants<Motor_type>::VEL_SCALE, MotorConstants<Motor_type>::VEL_SCALE);
        double torque_clamped = std::clamp(control_param.ffTorque , -MotorConstants<Motor_type>::TQ_SCALE, MotorConstants<Motor_type>::TQ_SCALE);

        uint16_t pos_u16 = (uint16_t)(((pos_clamped / MotorConstants<Motor_type>::POS_SCALE) + 1.0) * 0x7FFF);
        uint16_t vel_u16 = (uint16_t)(((vel_clamped / MotorConstants<Motor_type>::VEL_SCALE) + 1.0) * 0x7FFF);
        uint16_t kp_u16 = (uint16_t)((kp_clamped / MotorConstants<Motor_type>::KP_SCALE) * 0xFFFF);
        uint16_t kd_u16 = (uint16_t)((kd_clamped / MotorConstants<Motor_type>::KD_SCALE) * 0xFFFF);
        uint16_t torque_u16 = (uint16_t)(((torque_clamped / MotorConstants<Motor_type>::TQ_SCALE) + 1.0) * 0x7FFF);
    
        uint8_t data[8];
        pack_u16_be(&data[0], pos_u16);
        pack_u16_be(&data[2], vel_u16);
        pack_u16_be(&data[4], kp_u16);
        pack_u16_be(&data[6], kd_u16);
        
            // 2. Construct CAN ID
            //uint32_t ext_id = (COMM_OPERATION_CONTROL << 24) | (torque_u16 << 8) | motor_id;      
        uint32_t ext_id = (COMM_OPERATION_CONTROL << 24) | ((0xffff&torque_u16) << 8) | can_id; // 정수형으로 변환되어 연산되는 경우가 잦으므로 비트마스킹을 하는것이 좋은 습관 
        
            // 3. Send
        return send_frame(ext_id, data, 8);
    
    }

    bool read_operation_frame(int s) {
        struct can_frame frame;
        if (read_frame(s, &frame)) {
            if (!(frame.can_id & CAN_EFF_FLAG)) return false;
            
            uint32_t comm_type = (frame.can_id >> 24) & 0x1F;
            if (comm_type == 2) { // Status packet
                return true;
            }
        }
        return false;
    }
    
    bool init_motor_MIT(float vel_lim, float torque_lim){
        try{
            enable_motor();
            usleep(1000);
            set_mode_raw(0);
            usleep(1000);
            write_limit(PARAM_VELOCITY_LIMIT, vel_lim);
            usleep(1000);
            write_limit(PARAM_TORQUE_LIMIT, torque_lim);

            return true;
        }catch(const std::exception& e){

            // 모터를 끄는 로직이 들어갑시다 
            return false;
        }

    }


    void write_FB(float pos, float vel , float torq_raw , float temp){
        Feedback_param.pos = pos*MotorConstants<Motor_type>::POS_SCALE;
        Feedback_param.vel = vel*MotorConstants<Motor_type>::VEL_SCALE;
        Feedback_param.torque = torq_raw*MotorConstants<Motor_type>::TQ_SCALE;
        Feedback_param.temp = temp;
    }


};


using Motortype = std::variant<RobstrideMotor<RobstrideMotor_type::RS00> , RobstrideMotor<RobstrideMotor_type::RS01>,RobstrideMotor<RobstrideMotor_type::RS02> ,RobstrideMotor<RobstrideMotor_type::RS03> ,RobstrideMotor<RobstrideMotor_type::RS04> ,RobstrideMotor<RobstrideMotor_type::RS05>,RobstrideMotor<RobstrideMotor_type::RS06>,RobstrideMotor<RobstrideMotor_type::EL05>>;
 // 
using Motor_con = std::tuple<std::vector<RobstrideMotor<RobstrideMotor_type::RS00>>, std::vector<RobstrideMotor<RobstrideMotor_type::RS01>>,
std::vector<RobstrideMotor<RobstrideMotor_type::RS02>> ,std::vector<RobstrideMotor<RobstrideMotor_type::RS03>>
 ,std::vector<RobstrideMotor<RobstrideMotor_type::RS04>> ,std::vector<RobstrideMotor<RobstrideMotor_type::RS05>>,
 std::vector<RobstrideMotor<RobstrideMotor_type::RS06>>,std::vector<RobstrideMotor<RobstrideMotor_type::EL05>>>;

 
 using RS00_Vec = std::vector<RobstrideMotor<RobstrideMotor_type::RS00>>; 
 using RS01_Vec = std::vector<RobstrideMotor<RobstrideMotor_type::RS01>>; 
 using RS02_Vec = std::vector<RobstrideMotor<RobstrideMotor_type::RS02>>; 
 using RS03_Vec = std::vector<RobstrideMotor<RobstrideMotor_type::RS03>>; 
 using RS04_Vec = std::vector<RobstrideMotor<RobstrideMotor_type::RS04>>; 
 using RS05_Vec = std::vector<RobstrideMotor<RobstrideMotor_type::RS05>>; 
 using RS06_Vec = std::vector<RobstrideMotor<RobstrideMotor_type::RS06>>; 
 using EL05_Vec = std::vector<RobstrideMotor<RobstrideMotor_type::EL05>>; 


int init_can(const char* interface) ;

bool readframe(int s, struct can_frame* frame);  // 이 이름으로 프레임 데이터 읽는 함수 작성 
                                                //아마 읽어오는 클래스를 하나 만드는게 좋을듯



#endif