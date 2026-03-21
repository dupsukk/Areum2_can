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


typedef enum RobstrideMotor_type {RS02 = 0, RS00 =1 , EL05 = 2};

typedef struct 
{
    uint16_t pos;
    uint16_t vel;
    uint16_t Kp;
    uint16_t Kd;
    uint16_t ffTorque;
}Control_param;


class RobstrideMotor {
    private :
    const RobstrideMotor_type Motor_type;
    const int can_id;
    const int can_interface;


    public:
    Control_param control_param;

    RobstrideMotor(RobstrideMotor_type , int can_id );

    void pack_float_le(uint8_t* buf, float val);
    void pack_u16_le(uint8_t* buf, uint16_t val);
    void pack_u16_be(uint8_t* buf, uint16_t val);

    bool send_frame(int s, uint32_t can_id, const uint8_t* data, uint8_t dlc) ;
    bool read_frame(int s, struct can_frame* frame);
    bool enable_motor(int s, int motor_id); 
    bool set_mode_raw(int s, int motor_id, int8_t mode); 
    bool write_limit(int s, int motor_id, uint16_t param_id, float limit);
    bool write_operation_frame(int s, int motor_id, double pos, double kp_val, double kd_val);
    bool read_operation_frame(int s); 

};


int init_can(const char* interface) ;

#endif