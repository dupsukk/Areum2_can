#ifndef ROBSTRIDEMOTOR_H
#define ROBSTRIDEMOTOR_H

#include <stdint.h>
#include <stdlib.h>
#include <string>
#include <cstring>
#include <atomic>
#include <stdio.h>
#include <unistd.h>
#include <cmath>
#include <variant>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "Areum2.hpp"


const uint32_t COMM_WRITE_PARAMETER = 18;
const uint32_t COMM_ENABLE = 3;
const int HOST_ID = 0xFF;
const uint16_t PARAM_MODE = 0x7005;
const uint32_t COMM_OPERATION_CONTROL = 1;
const uint16_t PARAM_VELOCITY_LIMIT = 0x7017;
const uint16_t PARAM_TORQUE_LIMIT = 0x700B;           


typedef enum RobstrideMotor_type {RS00 = 0, RS01 =1 , RS02 =2 , RS03 = 3, RS04= 4, RS05 =5, RS06=6, EL05 = 7};

typedef struct 
{
    uint16_t pos;
    uint16_t vel;
    uint16_t Kp;
    uint16_t Kd;
    uint16_t ffTorque;
}Control_param;


template <RobstrideMotor_type Motor_type>
class RobstrideMotor {
    private :
    //const RobstrideMotor_type Motor_type;
    const int can_id;
    const int can_interface;


    public:
    Control_param control_param;


    RobstrideMotor(RobstrideMotor_type Motor_type, int can_interface,int can_id ): 
        can_interface(can_interface) ,can_id(can_id) {
    
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
    
    bool write_operation_frame(double pos, double kp_val, double kd_val) {
        // 1. Pack data (big-endian!)
        // These scaling values should be imported from table.py; hardcoded here for simplicity
                                                            //테이블에 있는거 가져오긴 했는데 이상하게 훨씬 약한 모터인데 17Nm를 스케일로 쓴다고?? 이상하네;; 
        if constexpr(Motor_type==RS00){
            const double POS_SCALE = 4 * M_PI; // rs-00
            const double VEL_SCALE = 50.0;     // rs-00
            const double KP_SCALE = 500.0;    // rs-00
            const double KD_SCALE = 5.0;     // rs-00
            const double TQ_SCALE = 17.0;      // rs-00

                    // Clamp and convert
            double pos_clamped = std::max(-POS_SCALE, std::min(POS_SCALE, pos));
            double kp_clamped = std::max(0.0, std::min(KP_SCALE, kp_val));
            double kd_clamped = std::max(0.0, std::min(KD_SCALE, kd_val));                // std::clamp로 바꾸고 싶다 그죠? 
        
            uint16_t pos_u16 = (uint16_t)(((pos_clamped / POS_SCALE) + 1.0) * 0x7FFF);
            uint16_t vel_u16 = 0x7FFF; // 0 velocity
            uint16_t kp_u16 = (uint16_t)((kp_clamped / KP_SCALE) * 0xFFFF);
            uint16_t kd_u16 = (uint16_t)((kd_clamped / KD_SCALE) * 0xFFFF);
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
        else if constexpr(Motor_type==RS01){
            const double POS_SCALE = 4 * M_PI; // rs-01
            const double VEL_SCALE = 44.0;     // rs-01
            const double KP_SCALE = 500.0;    // rs-01
            const double KD_SCALE = 5.0;     // rs-01
            const double TQ_SCALE = 17.0;      // rs-01

                    // Clamp and convert
            double pos_clamped = std::max(-POS_SCALE, std::min(POS_SCALE, pos));
            double kp_clamped = std::max(0.0, std::min(KP_SCALE, kp_val));
            double kd_clamped = std::max(0.0, std::min(KD_SCALE, kd_val));                // std::clamp로 바꾸고 싶다 그죠? 
            
            uint16_t pos_u16 = (uint16_t)(((pos_clamped / POS_SCALE) + 1.0) * 0x7FFF);
            uint16_t vel_u16 = 0x7FFF; // 0 velocity
            uint16_t kp_u16 = (uint16_t)((kp_clamped / KP_SCALE) * 0xFFFF);
            uint16_t kd_u16 = (uint16_t)((kd_clamped / KD_SCALE) * 0xFFFF);
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
        else if constexpr(Motor_type==RS02){
            const double POS_SCALE = 4 * M_PI; // rs-02
            const double VEL_SCALE = 44.0;     // rs-02
            const double KP_SCALE = 500.0;    // rs-02
            const double KD_SCALE = 5.0;     // rs-02
            const double TQ_SCALE = 17.0;      // rs-02

                    // Clamp and convert
            double pos_clamped = std::max(-POS_SCALE, std::min(POS_SCALE, pos));
            double kp_clamped = std::max(0.0, std::min(KP_SCALE, kp_val));
            double kd_clamped = std::max(0.0, std::min(KD_SCALE, kd_val));                // std::clamp로 바꾸고 싶다 그죠? 
        
            uint16_t pos_u16 = (uint16_t)(((pos_clamped / POS_SCALE) + 1.0) * 0x7FFF);
            uint16_t vel_u16 = 0x7FFF; // 0 velocity
            uint16_t kp_u16 = (uint16_t)((kp_clamped / KP_SCALE) * 0xFFFF);
            uint16_t kd_u16 = (uint16_t)((kd_clamped / KD_SCALE) * 0xFFFF);
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
        else if constexpr(Motor_type==RS03){
            const double POS_SCALE = 4 * M_PI; // rs-03
            const double VEL_SCALE = 50.0;     // rs-03
            const double KP_SCALE = 5000.0;    // rs-03
            const double KD_SCALE = 100.0;     // rs-03
            const double TQ_SCALE = 60.0;      // rs-03

                    // Clamp and convert
            double pos_clamped = std::max(-POS_SCALE, std::min(POS_SCALE, pos));
            double kp_clamped = std::max(0.0, std::min(KP_SCALE, kp_val));
            double kd_clamped = std::max(0.0, std::min(KD_SCALE, kd_val));                // std::clamp로 바꾸고 싶다 그죠? 
        
            uint16_t pos_u16 = (uint16_t)(((pos_clamped / POS_SCALE) + 1.0) * 0x7FFF);
            uint16_t vel_u16 = 0x7FFF; // 0 velocity
            uint16_t kp_u16 = (uint16_t)((kp_clamped / KP_SCALE) * 0xFFFF);
            uint16_t kd_u16 = (uint16_t)((kd_clamped / KD_SCALE) * 0xFFFF);
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
        else if constexpr(Motor_type==RS04){
            const double POS_SCALE = 4 * M_PI; // rs-04
            const double VEL_SCALE = 15.0;     // rs-04
            const double KP_SCALE = 5000.0;    // rs-04
            const double KD_SCALE = 100.0;     // rs-04
            const double TQ_SCALE = 120.0;      // rs-04

                    // Clamp and convert
            double pos_clamped = std::max(-POS_SCALE, std::min(POS_SCALE, pos));
            double kp_clamped = std::max(0.0, std::min(KP_SCALE, kp_val));
            double kd_clamped = std::max(0.0, std::min(KD_SCALE, kd_val));                // std::clamp로 바꾸고 싶다 그죠? 
        
            uint16_t pos_u16 = (uint16_t)(((pos_clamped / POS_SCALE) + 1.0) * 0x7FFF);
            uint16_t vel_u16 = 0x7FFF; // 0 velocity
            uint16_t kp_u16 = (uint16_t)((kp_clamped / KP_SCALE) * 0xFFFF);
            uint16_t kd_u16 = (uint16_t)((kd_clamped / KD_SCALE) * 0xFFFF);
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
        else if constexpr(Motor_type==RS05){
            const double POS_SCALE = 4 * M_PI; // rs-05
            const double VEL_SCALE = 33.0;     // rs-05
            const double KP_SCALE = 500.0;    // rs-05
            const double KD_SCALE = 5.0;     // rs-05
            const double TQ_SCALE = 17.0;      // rs-05

                    // Clamp and convert
            double pos_clamped = std::max(-POS_SCALE, std::min(POS_SCALE, pos));
            double kp_clamped = std::max(0.0, std::min(KP_SCALE, kp_val));
            double kd_clamped = std::max(0.0, std::min(KD_SCALE, kd_val));                // std::clamp로 바꾸고 싶다 그죠? 
        
            uint16_t pos_u16 = (uint16_t)(((pos_clamped / POS_SCALE) + 1.0) * 0x7FFF);
            uint16_t vel_u16 = 0x7FFF; // 0 velocity
            uint16_t kp_u16 = (uint16_t)((kp_clamped / KP_SCALE) * 0xFFFF);
            uint16_t kd_u16 = (uint16_t)((kd_clamped / KD_SCALE) * 0xFFFF);
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
        else if constexpr(Motor_type==RS06){
            const double POS_SCALE = 4 * M_PI; // rs-06
            const double VEL_SCALE = 20.0;     // rs-06
            const double KP_SCALE = 5000.0;    // rs-06
            const double KD_SCALE = 5.0;     // rs-06
            const double TQ_SCALE = 60.0;      // rs-06

                    // Clamp and convert
            double pos_clamped = std::max(-POS_SCALE, std::min(POS_SCALE, pos));
            double kp_clamped = std::max(0.0, std::min(KP_SCALE, kp_val));
            double kd_clamped = std::max(0.0, std::min(KD_SCALE, kd_val));                // std::clamp로 바꾸고 싶다 그죠? 
        
            uint16_t pos_u16 = (uint16_t)(((pos_clamped / POS_SCALE) + 1.0) * 0x7FFF);
            uint16_t vel_u16 = 0x7FFF; // 0 velocity
            uint16_t kp_u16 = (uint16_t)((kp_clamped / KP_SCALE) * 0xFFFF);
            uint16_t kd_u16 = (uint16_t)((kd_clamped / KD_SCALE) * 0xFFFF);
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
        else if constexpr(Motor_type==EL05){
            const double POS_SCALE = 4 * M_PI; // rs-05
            const double VEL_SCALE = 33.0;     // rs-05
            const double KP_SCALE = 500.0;    // rs-05
            const double KD_SCALE = 5.0;     // rs-05
            const double TQ_SCALE = 17.0;      // rs-05

                    // Clamp and convert
            double pos_clamped = std::max(-POS_SCALE, std::min(POS_SCALE, pos));
            double kp_clamped = std::max(0.0, std::min(KP_SCALE, kp_val));
            double kd_clamped = std::max(0.0, std::min(KD_SCALE, kd_val));                // std::clamp로 바꾸고 싶다 그죠? 
        
            uint16_t pos_u16 = (uint16_t)(((pos_clamped / POS_SCALE) + 1.0) * 0x7FFF);
            uint16_t vel_u16 = 0x7FFF; // 0 velocity
            uint16_t kp_u16 = (uint16_t)((kp_clamped / KP_SCALE) * 0xFFFF);
            uint16_t kd_u16 = (uint16_t)((kd_clamped / KD_SCALE) * 0xFFFF);
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


        return false;
    
    }

    bool read_operation_frame(int s) {
        struct can_frame frame;
        if (read_frame(s, &frame)) {
            if (!frame.can_id & CAN_EFF_FLAG) return false;
            
            uint32_t comm_type = (frame.can_id >> 24) & 0x1F;
            if (comm_type == 2) { // Status packet
                return true;
            }
        }
        return false;
    }
    
    

};


int init_can(const char* interface) ;

using Motortype = std::variant<RobstrideMotor<RS00> , RobstrideMotor<RS01>,RobstrideMotor<RS02> ,RobstrideMotor<RS03> ,RobstrideMotor<RS04> ,RobstrideMotor<RS05>,RobstrideMotor<RS06>,RobstrideMotor<EL05>>;



#endif