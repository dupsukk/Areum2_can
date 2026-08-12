#ifndef HUMANOID_CONFIG_HPP
#define HUMANOID_CONFIG_HPP

#include <cmath>

// CAN IDs 
#define CAN_ID_LEFT_HIP_PITCH     0x01
#define CAN_ID_LEFT_HIP_ROLL      0x02
#define CAN_ID_LEFT_HIP_YAW       0x03
#define CAN_ID_LEFT_KNEE_PITCH    0x04
#define CAN_ID_LEFT_ANKLE_A       0x05
#define CAN_ID_LEFT_ANKLE_B       0x06

#define CAN_ID_RIGHT_HIP_PITCH    0x11
#define CAN_ID_RIGHT_HIP_ROLL     0x12
#define CAN_ID_RIGHT_HIP_YAW      0x13
#define CAN_ID_RIGHT_KNEE_PITCH   0x14
#define CAN_ID_RIGHT_ANKLE_A      0x15
#define CAN_ID_RIGHT_ANKLE_B      0x16

// SHM INDEX
#define SHM_MOTOR_INDEX_LEFT_HIP_PITCH    0
#define SHM_MOTOR_INDEX_LEFT_HIP_ROLL     4
#define SHM_MOTOR_INDEX_LEFT_HIP_YAW      6
#define SHM_MOTOR_INDEX_LEFT_KNEE_PITCH   2
#define SHM_MOTOR_INDEX_LEFT_ANKLE_A      8
#define SHM_MOTOR_INDEX_LEFT_ANKLE_B      10

#define SHM_MOTOR_INDEX_RIGHT_HIP_PITCH   1
#define SHM_MOTOR_INDEX_RIGHT_HIP_ROLL    5
#define SHM_MOTOR_INDEX_IGHT_HIP_YAW      7
#define SHM_MOTOR_INDEX_RIGHT_KNEE_PITCH  3
#define SHM_MOTOR_INDEX_RIGHT_ANKLE_A     9
#define SHM_MOTOR_INDEX_RIGHT_ANKLE_B     11

//LEFT LEG LIMITS - deg 2 rad 
constexpr double MAX_POS_LEFT_HIP_PITCH   = 80.0    *  M_PI/180; //130 
constexpr double MIN_POS_LEFT_HIP_PITCH   = -80.0    *  M_PI/180; //-80
constexpr double MAX_POS_LEFT_HIP_ROLL    = 8        *  M_PI/180; //8
constexpr double MIN_POS_LEFT_HIP_ROLL    = -15      *  M_PI/180; // 원래 -90
constexpr double MAX_POS_LEFT_HIP_YAW     = 45       *  M_PI/180; //90
constexpr double MIN_POS_LEFT_HIP_YAW     = -45      *  M_PI/180; //-90
constexpr double MAX_POS_LEFT_KNEE_PITCH  = 0        *  M_PI/180; //0
constexpr double MIN_POS_LEFT_KNEE_PITCH  = -90     *  M_PI/180; //-110
constexpr double MAX_POS_LEFT_ANKLE_A     = 65.5     *  M_PI/180; //65.5
constexpr double MIN_POS_LEFT_ANKLE_A     = -41      *  M_PI/180; //-41
constexpr double MAX_POS_LEFT_ANKLE_B     = 41.5     *  M_PI/180; //41.5
constexpr double MIN_POS_LEFT_ANKLE_B     = -65.5    *  M_PI/180; //-65.5 부호 확인 필요 

//RIGHT LEG LIMITS - deg 2 rad
constexpr double MAX_POS_RIGHT_HIP_PITCH   = 80.0     *  M_PI/180; //80
constexpr double MIN_POS_RIGHT_HIP_PITCH   = -80.0   *  M_PI/180; //-130
constexpr double MAX_POS_RIGHT_HIP_ROLL    = 25       *  M_PI/180; //원래 90 
constexpr double MIN_POS_RIGHT_HIP_ROLL    = -8       *  M_PI/180; //-8
constexpr double MAX_POS_RIGHT_HIP_YAW     = 45       *  M_PI/180; //90
constexpr double MIN_POS_RIGHT_HIP_YAW     = -45      *  M_PI/180; //-90
constexpr double MAX_POS_RIGHT_KNEE_PITCH  = 90      *  M_PI/180; //110
constexpr double MIN_POS_RIGHT_KNEE_PITCH  = 0        *  M_PI/180; //0
constexpr double MAX_POS_RIGHT_ANKLE_A     = 0        *  M_PI/180; //0
constexpr double MIN_POS_RIGHT_ANKLE_A     = 65.5     *  M_PI/180; //65.5
constexpr double MAX_POS_RIGHT_ANKLE_B     = 41.0     *  M_PI/180; //41
constexpr double MIN_POS_RIGHT_ANKLE_B     = -65.5    *  M_PI/180; //-65.5 부호 확인 필요 

#endif
