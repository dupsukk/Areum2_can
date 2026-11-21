// 자동 생성 -- 고치지 말 것.  출처: robot.yaml  (생성기: gen_robot.py)
// YAML sha256[:16] = 5660e2c54f5f5f22
#ifndef AREUMII_CONFIG_H
#define AREUMII_CONFIG_H

#define ROBOT_CONFIG_INCLUDED  // can_runtime.hpp 가 include 순서를 검사하는 용도

#include <cstdint>
#include <tuple>
#include "RobstrideMotor.hpp"  // Motor_con, *_Vec

// MIT: write_updated_operation_frame (pos/vel/Kp/Kd),  PP: write_pp_position (목표 위치만)
enum class ControlMode { MIT, PP };

inline constexpr const char* ROBOT_NAME = "AReuMii";
inline constexpr const char* ROBOT_YAML_SHA = "5660e2c54f5f5f22";
inline constexpr long CONTROL_PERIOD_NS = 3000000;
inline constexpr int SHM_SYNC_PERIOD_US = 1000;
inline constexpr int ROBOT_SHM_KEY = 13563267;
inline constexpr int MOTOR_NUM = 16;
inline constexpr int CAN_MAX_ID = 25;  // 최대 can_id + 1 (Rx_handler 테이블 크기)
inline constexpr double INIT_POS_TOLERANCE_RAD = 0.3490658503988659;  // 시작 시 영점 허용 편차 (20.0 deg)
inline constexpr int CAN_INTERFACE_NUM = 2;
inline constexpr const char* CAN_INTERFACE_NAMES[CAN_INTERFACE_NUM] = {"can0", "can1"};

// CAN ID
inline constexpr uint32_t CAN_ID_LEFT_SHOULDER_PITCH  = 0x01;
inline constexpr uint32_t CAN_ID_LEFT_SHOULDER_ROLL   = 0x02;
inline constexpr uint32_t CAN_ID_LEFT_SHOULDER_YAW    = 0x03;
inline constexpr uint32_t CAN_ID_LEFT_ELBOW_PITCH     = 0x04;
inline constexpr uint32_t CAN_ID_LEFT_WRIST_ROLL      = 0x05;
inline constexpr uint32_t CAN_ID_LEFT_WRIST_YAW       = 0x06;
inline constexpr uint32_t CAN_ID_LEFT_WRIST_PITCH     = 0x07;
inline constexpr uint32_t CAN_ID_LEFT_GRIPPER         = 0x08;
inline constexpr uint32_t CAN_ID_RIGHT_SHOULDER_PITCH = 0x11;
inline constexpr uint32_t CAN_ID_RIGHT_SHOULDER_ROLL  = 0x12;
inline constexpr uint32_t CAN_ID_RIGHT_SHOULDER_YAW   = 0x13;
inline constexpr uint32_t CAN_ID_RIGHT_ELBOW_PITCH    = 0x14;
inline constexpr uint32_t CAN_ID_RIGHT_WRIST_ROLL     = 0x15;
inline constexpr uint32_t CAN_ID_RIGHT_WRIST_YAW      = 0x16;
inline constexpr uint32_t CAN_ID_RIGHT_WRIST_PITCH    = 0x17;
inline constexpr uint32_t CAN_ID_RIGHT_GRIPPER        = 0x18;

// shared memory index
inline constexpr int SHM_INDEX_LEFT_SHOULDER_PITCH  = 0;
inline constexpr int SHM_INDEX_LEFT_SHOULDER_ROLL   = 1;
inline constexpr int SHM_INDEX_LEFT_SHOULDER_YAW    = 2;
inline constexpr int SHM_INDEX_LEFT_ELBOW_PITCH     = 3;
inline constexpr int SHM_INDEX_LEFT_WRIST_ROLL      = 4;
inline constexpr int SHM_INDEX_LEFT_WRIST_YAW       = 5;
inline constexpr int SHM_INDEX_LEFT_WRIST_PITCH     = 6;
inline constexpr int SHM_INDEX_LEFT_GRIPPER         = 7;
inline constexpr int SHM_INDEX_RIGHT_SHOULDER_PITCH = 8;
inline constexpr int SHM_INDEX_RIGHT_SHOULDER_ROLL  = 9;
inline constexpr int SHM_INDEX_RIGHT_SHOULDER_YAW   = 10;
inline constexpr int SHM_INDEX_RIGHT_ELBOW_PITCH    = 11;
inline constexpr int SHM_INDEX_RIGHT_WRIST_ROLL     = 12;
inline constexpr int SHM_INDEX_RIGHT_WRIST_YAW      = 13;
inline constexpr int SHM_INDEX_RIGHT_WRIST_PITCH    = 14;
inline constexpr int SHM_INDEX_RIGHT_GRIPPER        = 15;

// vector index (타입별 deque 내 순번)
inline constexpr int VEC_INDEX_LEFT_SHOULDER_PITCH  = 0;
inline constexpr int VEC_INDEX_LEFT_SHOULDER_ROLL   = 1;
inline constexpr int VEC_INDEX_LEFT_SHOULDER_YAW    = 0;
inline constexpr int VEC_INDEX_LEFT_ELBOW_PITCH     = 1;
inline constexpr int VEC_INDEX_LEFT_WRIST_ROLL      = 0;
inline constexpr int VEC_INDEX_LEFT_WRIST_YAW       = 1;
inline constexpr int VEC_INDEX_LEFT_WRIST_PITCH     = 2;
inline constexpr int VEC_INDEX_LEFT_GRIPPER         = 3;
inline constexpr int VEC_INDEX_RIGHT_SHOULDER_PITCH = 2;
inline constexpr int VEC_INDEX_RIGHT_SHOULDER_ROLL  = 3;
inline constexpr int VEC_INDEX_RIGHT_SHOULDER_YAW   = 2;
inline constexpr int VEC_INDEX_RIGHT_ELBOW_PITCH    = 3;
inline constexpr int VEC_INDEX_RIGHT_WRIST_ROLL     = 4;
inline constexpr int VEC_INDEX_RIGHT_WRIST_YAW      = 5;
inline constexpr int VEC_INDEX_RIGHT_WRIST_PITCH    = 6;
inline constexpr int VEC_INDEX_RIGHT_GRIPPER        = 7;

// CAN interface index (CAN_INTERFACE_NAMES 의 인덱스)
inline constexpr int CAN_IF_INDEX_LEFT_SHOULDER_PITCH  = 0;
inline constexpr int CAN_IF_INDEX_LEFT_SHOULDER_ROLL   = 0;
inline constexpr int CAN_IF_INDEX_LEFT_SHOULDER_YAW    = 0;
inline constexpr int CAN_IF_INDEX_LEFT_ELBOW_PITCH     = 0;
inline constexpr int CAN_IF_INDEX_LEFT_WRIST_ROLL      = 0;
inline constexpr int CAN_IF_INDEX_LEFT_WRIST_YAW       = 0;
inline constexpr int CAN_IF_INDEX_LEFT_WRIST_PITCH     = 0;
inline constexpr int CAN_IF_INDEX_LEFT_GRIPPER         = 0;
inline constexpr int CAN_IF_INDEX_RIGHT_SHOULDER_PITCH = 1;
inline constexpr int CAN_IF_INDEX_RIGHT_SHOULDER_ROLL  = 1;
inline constexpr int CAN_IF_INDEX_RIGHT_SHOULDER_YAW   = 1;
inline constexpr int CAN_IF_INDEX_RIGHT_ELBOW_PITCH    = 1;
inline constexpr int CAN_IF_INDEX_RIGHT_WRIST_ROLL     = 1;
inline constexpr int CAN_IF_INDEX_RIGHT_WRIST_YAW      = 1;
inline constexpr int CAN_IF_INDEX_RIGHT_WRIST_PITCH    = 1;
inline constexpr int CAN_IF_INDEX_RIGHT_GRIPPER        = 1;

// 방향 부호 (모터 -> 관절 좌표계)
inline constexpr double SIGN_LEFT_SHOULDER_PITCH  = -1.0;
inline constexpr double SIGN_LEFT_SHOULDER_ROLL   = 1.0;
inline constexpr double SIGN_LEFT_SHOULDER_YAW    = -1.0;
inline constexpr double SIGN_LEFT_ELBOW_PITCH     = -1.0;
inline constexpr double SIGN_LEFT_WRIST_ROLL      = -1.0;
inline constexpr double SIGN_LEFT_WRIST_YAW       = 1.0;
inline constexpr double SIGN_LEFT_WRIST_PITCH     = 1.0;
inline constexpr double SIGN_LEFT_GRIPPER         = -1.0;
inline constexpr double SIGN_RIGHT_SHOULDER_PITCH = 1.0;
inline constexpr double SIGN_RIGHT_SHOULDER_ROLL  = -1.0;
inline constexpr double SIGN_RIGHT_SHOULDER_YAW   = 1.0;
inline constexpr double SIGN_RIGHT_ELBOW_PITCH    = 1.0;
inline constexpr double SIGN_RIGHT_WRIST_ROLL     = 1.0;
inline constexpr double SIGN_RIGHT_WRIST_YAW      = -1.0;
inline constexpr double SIGN_RIGHT_WRIST_PITCH    = -1.0;
inline constexpr double SIGN_RIGHT_GRIPPER        = 1.0;

// 기계적 오프셋 [rad] (부호 적용된 관절 좌표계)
inline constexpr double MECH_OFFSET_LEFT_SHOULDER_PITCH  = 0.0;
inline constexpr double MECH_OFFSET_LEFT_SHOULDER_ROLL   = 0.0;
inline constexpr double MECH_OFFSET_LEFT_SHOULDER_YAW    = 0.0;
inline constexpr double MECH_OFFSET_LEFT_ELBOW_PITCH     = 0.0;
inline constexpr double MECH_OFFSET_LEFT_WRIST_ROLL      = 0.0;
inline constexpr double MECH_OFFSET_LEFT_WRIST_YAW       = 0.0;
inline constexpr double MECH_OFFSET_LEFT_WRIST_PITCH     = 0.0;
inline constexpr double MECH_OFFSET_LEFT_GRIPPER         = 0.0;
inline constexpr double MECH_OFFSET_RIGHT_SHOULDER_PITCH = 0.0;
inline constexpr double MECH_OFFSET_RIGHT_SHOULDER_ROLL  = 0.0;
inline constexpr double MECH_OFFSET_RIGHT_SHOULDER_YAW   = 0.0;
inline constexpr double MECH_OFFSET_RIGHT_ELBOW_PITCH    = 0.0;
inline constexpr double MECH_OFFSET_RIGHT_WRIST_ROLL     = 0.0;
inline constexpr double MECH_OFFSET_RIGHT_WRIST_YAW      = 0.0;
inline constexpr double MECH_OFFSET_RIGHT_WRIST_PITCH    = 0.0;
inline constexpr double MECH_OFFSET_RIGHT_GRIPPER        = 0.0;

// 제어 모드
inline constexpr ControlMode MODE_LEFT_SHOULDER_PITCH  = ControlMode::PP;
inline constexpr ControlMode MODE_LEFT_SHOULDER_ROLL   = ControlMode::PP;
inline constexpr ControlMode MODE_LEFT_SHOULDER_YAW    = ControlMode::PP;
inline constexpr ControlMode MODE_LEFT_ELBOW_PITCH     = ControlMode::PP;
inline constexpr ControlMode MODE_LEFT_WRIST_ROLL      = ControlMode::PP;
inline constexpr ControlMode MODE_LEFT_WRIST_YAW       = ControlMode::PP;
inline constexpr ControlMode MODE_LEFT_WRIST_PITCH     = ControlMode::PP;
inline constexpr ControlMode MODE_LEFT_GRIPPER         = ControlMode::MIT;
inline constexpr ControlMode MODE_RIGHT_SHOULDER_PITCH = ControlMode::PP;
inline constexpr ControlMode MODE_RIGHT_SHOULDER_ROLL  = ControlMode::PP;
inline constexpr ControlMode MODE_RIGHT_SHOULDER_YAW   = ControlMode::PP;
inline constexpr ControlMode MODE_RIGHT_ELBOW_PITCH    = ControlMode::PP;
inline constexpr ControlMode MODE_RIGHT_WRIST_ROLL     = ControlMode::PP;
inline constexpr ControlMode MODE_RIGHT_WRIST_YAW      = ControlMode::PP;
inline constexpr ControlMode MODE_RIGHT_WRIST_PITCH    = ControlMode::PP;
inline constexpr ControlMode MODE_RIGHT_GRIPPER        = ControlMode::MIT;

// init_motor_MIT 제한 (속도 [rad/s], 토크 [Nm]). PP 모터는 쓰지 않음(0)
inline constexpr double VEL_LIMIT_LEFT_SHOULDER_PITCH  = 0.0;
inline constexpr double TORQUE_LIMIT_LEFT_SHOULDER_PITCH  = 0.0;
inline constexpr double VEL_LIMIT_LEFT_SHOULDER_ROLL   = 0.0;
inline constexpr double TORQUE_LIMIT_LEFT_SHOULDER_ROLL   = 0.0;
inline constexpr double VEL_LIMIT_LEFT_SHOULDER_YAW    = 0.0;
inline constexpr double TORQUE_LIMIT_LEFT_SHOULDER_YAW    = 0.0;
inline constexpr double VEL_LIMIT_LEFT_ELBOW_PITCH     = 0.0;
inline constexpr double TORQUE_LIMIT_LEFT_ELBOW_PITCH     = 0.0;
inline constexpr double VEL_LIMIT_LEFT_WRIST_ROLL      = 0.0;
inline constexpr double TORQUE_LIMIT_LEFT_WRIST_ROLL      = 0.0;
inline constexpr double VEL_LIMIT_LEFT_WRIST_YAW       = 0.0;
inline constexpr double TORQUE_LIMIT_LEFT_WRIST_YAW       = 0.0;
inline constexpr double VEL_LIMIT_LEFT_WRIST_PITCH     = 0.0;
inline constexpr double TORQUE_LIMIT_LEFT_WRIST_PITCH     = 0.0;
inline constexpr double VEL_LIMIT_LEFT_GRIPPER         = 10.0;
inline constexpr double TORQUE_LIMIT_LEFT_GRIPPER         = 10.0;
inline constexpr double VEL_LIMIT_RIGHT_SHOULDER_PITCH = 0.0;
inline constexpr double TORQUE_LIMIT_RIGHT_SHOULDER_PITCH = 0.0;
inline constexpr double VEL_LIMIT_RIGHT_SHOULDER_ROLL  = 0.0;
inline constexpr double TORQUE_LIMIT_RIGHT_SHOULDER_ROLL  = 0.0;
inline constexpr double VEL_LIMIT_RIGHT_SHOULDER_YAW   = 0.0;
inline constexpr double TORQUE_LIMIT_RIGHT_SHOULDER_YAW   = 0.0;
inline constexpr double VEL_LIMIT_RIGHT_ELBOW_PITCH    = 0.0;
inline constexpr double TORQUE_LIMIT_RIGHT_ELBOW_PITCH    = 0.0;
inline constexpr double VEL_LIMIT_RIGHT_WRIST_ROLL     = 0.0;
inline constexpr double TORQUE_LIMIT_RIGHT_WRIST_ROLL     = 0.0;
inline constexpr double VEL_LIMIT_RIGHT_WRIST_YAW      = 0.0;
inline constexpr double TORQUE_LIMIT_RIGHT_WRIST_YAW      = 0.0;
inline constexpr double VEL_LIMIT_RIGHT_WRIST_PITCH    = 0.0;
inline constexpr double TORQUE_LIMIT_RIGHT_WRIST_PITCH    = 0.0;
inline constexpr double VEL_LIMIT_RIGHT_GRIPPER        = 10.0;
inline constexpr double TORQUE_LIMIT_RIGHT_GRIPPER        = 10.0;

// 위치 한계 [rad] (shm 좌표계)
inline constexpr double MIN_POS_LEFT_SHOULDER_PITCH  = -2.0;
inline constexpr double MAX_POS_LEFT_SHOULDER_PITCH  = 2.0;
inline constexpr double MIN_POS_LEFT_SHOULDER_ROLL   = -2.0;
inline constexpr double MAX_POS_LEFT_SHOULDER_ROLL   = 2.0;
inline constexpr double MIN_POS_LEFT_SHOULDER_YAW    = -2.0;
inline constexpr double MAX_POS_LEFT_SHOULDER_YAW    = 2.0;
inline constexpr double MIN_POS_LEFT_ELBOW_PITCH     = -2.0;
inline constexpr double MAX_POS_LEFT_ELBOW_PITCH     = 2.0;
inline constexpr double MIN_POS_LEFT_WRIST_ROLL      = -2.0;
inline constexpr double MAX_POS_LEFT_WRIST_ROLL      = 2.0;
inline constexpr double MIN_POS_LEFT_WRIST_YAW       = -2.0;
inline constexpr double MAX_POS_LEFT_WRIST_YAW       = 2.0;
inline constexpr double MIN_POS_LEFT_WRIST_PITCH     = -2.0;
inline constexpr double MAX_POS_LEFT_WRIST_PITCH     = 2.0;
inline constexpr double MIN_POS_LEFT_GRIPPER         = -2.0;
inline constexpr double MAX_POS_LEFT_GRIPPER         = 2.0;
inline constexpr double MIN_POS_RIGHT_SHOULDER_PITCH = -2.0;
inline constexpr double MAX_POS_RIGHT_SHOULDER_PITCH = 2.0;
inline constexpr double MIN_POS_RIGHT_SHOULDER_ROLL  = -2.0;
inline constexpr double MAX_POS_RIGHT_SHOULDER_ROLL  = 0.3;
inline constexpr double MIN_POS_RIGHT_SHOULDER_YAW   = -1.5;
inline constexpr double MAX_POS_RIGHT_SHOULDER_YAW   = 1.5;
inline constexpr double MIN_POS_RIGHT_ELBOW_PITCH    = -2.0;
inline constexpr double MAX_POS_RIGHT_ELBOW_PITCH    = 2.0;
inline constexpr double MIN_POS_RIGHT_WRIST_ROLL     = -2.0;
inline constexpr double MAX_POS_RIGHT_WRIST_ROLL     = 2.0;
inline constexpr double MIN_POS_RIGHT_WRIST_YAW      = -2.0;
inline constexpr double MAX_POS_RIGHT_WRIST_YAW      = 2.0;
inline constexpr double MIN_POS_RIGHT_WRIST_PITCH    = -2.0;
inline constexpr double MAX_POS_RIGHT_WRIST_PITCH    = 2.0;
inline constexpr double MIN_POS_RIGHT_GRIPPER        = -2.0;
inline constexpr double MAX_POS_RIGHT_GRIPPER        = 2.0;

// PP 파라미터 (속도 [rad/s], 가속도 [rad/s^2], 전류 제한 [A]). MIT 모터는 쓰지 않음(0)
inline constexpr double PP_VEL_MAX_LEFT_SHOULDER_PITCH  = 2.0;
inline constexpr double PP_ACC_LEFT_SHOULDER_PITCH  = 10.0;
inline constexpr double PP_CURRENT_LIMIT_LEFT_SHOULDER_PITCH  = 10.0;
inline constexpr double PP_VEL_MAX_LEFT_SHOULDER_ROLL   = 2.0;
inline constexpr double PP_ACC_LEFT_SHOULDER_ROLL   = 10.0;
inline constexpr double PP_CURRENT_LIMIT_LEFT_SHOULDER_ROLL   = 10.0;
inline constexpr double PP_VEL_MAX_LEFT_SHOULDER_YAW    = 2.0;
inline constexpr double PP_ACC_LEFT_SHOULDER_YAW    = 10.0;
inline constexpr double PP_CURRENT_LIMIT_LEFT_SHOULDER_YAW    = 10.0;
inline constexpr double PP_VEL_MAX_LEFT_ELBOW_PITCH     = 2.0;
inline constexpr double PP_ACC_LEFT_ELBOW_PITCH     = 10.0;
inline constexpr double PP_CURRENT_LIMIT_LEFT_ELBOW_PITCH     = 10.0;
inline constexpr double PP_VEL_MAX_LEFT_WRIST_ROLL      = 2.0;
inline constexpr double PP_ACC_LEFT_WRIST_ROLL      = 10.0;
inline constexpr double PP_CURRENT_LIMIT_LEFT_WRIST_ROLL      = 10.0;
inline constexpr double PP_VEL_MAX_LEFT_WRIST_YAW       = 2.0;
inline constexpr double PP_ACC_LEFT_WRIST_YAW       = 10.0;
inline constexpr double PP_CURRENT_LIMIT_LEFT_WRIST_YAW       = 10.0;
inline constexpr double PP_VEL_MAX_LEFT_WRIST_PITCH     = 2.0;
inline constexpr double PP_ACC_LEFT_WRIST_PITCH     = 10.0;
inline constexpr double PP_CURRENT_LIMIT_LEFT_WRIST_PITCH     = 10.0;
inline constexpr double PP_VEL_MAX_LEFT_GRIPPER         = 0.0;
inline constexpr double PP_ACC_LEFT_GRIPPER         = 0.0;
inline constexpr double PP_CURRENT_LIMIT_LEFT_GRIPPER         = 0.0;
inline constexpr double PP_VEL_MAX_RIGHT_SHOULDER_PITCH = 2.0;
inline constexpr double PP_ACC_RIGHT_SHOULDER_PITCH = 10.0;
inline constexpr double PP_CURRENT_LIMIT_RIGHT_SHOULDER_PITCH = 10.0;
inline constexpr double PP_VEL_MAX_RIGHT_SHOULDER_ROLL  = 2.0;
inline constexpr double PP_ACC_RIGHT_SHOULDER_ROLL  = 10.0;
inline constexpr double PP_CURRENT_LIMIT_RIGHT_SHOULDER_ROLL  = 10.0;
inline constexpr double PP_VEL_MAX_RIGHT_SHOULDER_YAW   = 2.0;
inline constexpr double PP_ACC_RIGHT_SHOULDER_YAW   = 10.0;
inline constexpr double PP_CURRENT_LIMIT_RIGHT_SHOULDER_YAW   = 10.0;
inline constexpr double PP_VEL_MAX_RIGHT_ELBOW_PITCH    = 2.0;
inline constexpr double PP_ACC_RIGHT_ELBOW_PITCH    = 10.0;
inline constexpr double PP_CURRENT_LIMIT_RIGHT_ELBOW_PITCH    = 10.0;
inline constexpr double PP_VEL_MAX_RIGHT_WRIST_ROLL     = 2.0;
inline constexpr double PP_ACC_RIGHT_WRIST_ROLL     = 10.0;
inline constexpr double PP_CURRENT_LIMIT_RIGHT_WRIST_ROLL     = 10.0;
inline constexpr double PP_VEL_MAX_RIGHT_WRIST_YAW      = 2.0;
inline constexpr double PP_ACC_RIGHT_WRIST_YAW      = 10.0;
inline constexpr double PP_CURRENT_LIMIT_RIGHT_WRIST_YAW      = 10.0;
inline constexpr double PP_VEL_MAX_RIGHT_WRIST_PITCH    = 2.0;
inline constexpr double PP_ACC_RIGHT_WRIST_PITCH    = 10.0;
inline constexpr double PP_CURRENT_LIMIT_RIGHT_WRIST_PITCH    = 10.0;
inline constexpr double PP_VEL_MAX_RIGHT_GRIPPER        = 0.0;
inline constexpr double PP_ACC_RIGHT_GRIPPER        = 0.0;
inline constexpr double PP_CURRENT_LIMIT_RIGHT_GRIPPER        = 0.0;

// shm 인덱스로 조회하는 표 (can_runtime.hpp 의 루프가 사용)
inline constexpr ControlMode SHM_MODE[MOTOR_NUM] = {MODE_LEFT_SHOULDER_PITCH, MODE_LEFT_SHOULDER_ROLL, MODE_LEFT_SHOULDER_YAW, MODE_LEFT_ELBOW_PITCH, MODE_LEFT_WRIST_ROLL, MODE_LEFT_WRIST_YAW, MODE_LEFT_WRIST_PITCH, MODE_LEFT_GRIPPER, MODE_RIGHT_SHOULDER_PITCH, MODE_RIGHT_SHOULDER_ROLL, MODE_RIGHT_SHOULDER_YAW, MODE_RIGHT_ELBOW_PITCH, MODE_RIGHT_WRIST_ROLL, MODE_RIGHT_WRIST_YAW, MODE_RIGHT_WRIST_PITCH, MODE_RIGHT_GRIPPER};
inline constexpr double SHM_SIGN[MOTOR_NUM] = {SIGN_LEFT_SHOULDER_PITCH, SIGN_LEFT_SHOULDER_ROLL, SIGN_LEFT_SHOULDER_YAW, SIGN_LEFT_ELBOW_PITCH, SIGN_LEFT_WRIST_ROLL, SIGN_LEFT_WRIST_YAW, SIGN_LEFT_WRIST_PITCH, SIGN_LEFT_GRIPPER, SIGN_RIGHT_SHOULDER_PITCH, SIGN_RIGHT_SHOULDER_ROLL, SIGN_RIGHT_SHOULDER_YAW, SIGN_RIGHT_ELBOW_PITCH, SIGN_RIGHT_WRIST_ROLL, SIGN_RIGHT_WRIST_YAW, SIGN_RIGHT_WRIST_PITCH, SIGN_RIGHT_GRIPPER};
inline constexpr double SHM_MECH_OFFSET[MOTOR_NUM] = {MECH_OFFSET_LEFT_SHOULDER_PITCH, MECH_OFFSET_LEFT_SHOULDER_ROLL, MECH_OFFSET_LEFT_SHOULDER_YAW, MECH_OFFSET_LEFT_ELBOW_PITCH, MECH_OFFSET_LEFT_WRIST_ROLL, MECH_OFFSET_LEFT_WRIST_YAW, MECH_OFFSET_LEFT_WRIST_PITCH, MECH_OFFSET_LEFT_GRIPPER, MECH_OFFSET_RIGHT_SHOULDER_PITCH, MECH_OFFSET_RIGHT_SHOULDER_ROLL, MECH_OFFSET_RIGHT_SHOULDER_YAW, MECH_OFFSET_RIGHT_ELBOW_PITCH, MECH_OFFSET_RIGHT_WRIST_ROLL, MECH_OFFSET_RIGHT_WRIST_YAW, MECH_OFFSET_RIGHT_WRIST_PITCH, MECH_OFFSET_RIGHT_GRIPPER};
inline constexpr double SHM_MIN_POS[MOTOR_NUM] = {MIN_POS_LEFT_SHOULDER_PITCH, MIN_POS_LEFT_SHOULDER_ROLL, MIN_POS_LEFT_SHOULDER_YAW, MIN_POS_LEFT_ELBOW_PITCH, MIN_POS_LEFT_WRIST_ROLL, MIN_POS_LEFT_WRIST_YAW, MIN_POS_LEFT_WRIST_PITCH, MIN_POS_LEFT_GRIPPER, MIN_POS_RIGHT_SHOULDER_PITCH, MIN_POS_RIGHT_SHOULDER_ROLL, MIN_POS_RIGHT_SHOULDER_YAW, MIN_POS_RIGHT_ELBOW_PITCH, MIN_POS_RIGHT_WRIST_ROLL, MIN_POS_RIGHT_WRIST_YAW, MIN_POS_RIGHT_WRIST_PITCH, MIN_POS_RIGHT_GRIPPER};
inline constexpr double SHM_MAX_POS[MOTOR_NUM] = {MAX_POS_LEFT_SHOULDER_PITCH, MAX_POS_LEFT_SHOULDER_ROLL, MAX_POS_LEFT_SHOULDER_YAW, MAX_POS_LEFT_ELBOW_PITCH, MAX_POS_LEFT_WRIST_ROLL, MAX_POS_LEFT_WRIST_YAW, MAX_POS_LEFT_WRIST_PITCH, MAX_POS_LEFT_GRIPPER, MAX_POS_RIGHT_SHOULDER_PITCH, MAX_POS_RIGHT_SHOULDER_ROLL, MAX_POS_RIGHT_SHOULDER_YAW, MAX_POS_RIGHT_ELBOW_PITCH, MAX_POS_RIGHT_WRIST_ROLL, MAX_POS_RIGHT_WRIST_YAW, MAX_POS_RIGHT_WRIST_PITCH, MAX_POS_RIGHT_GRIPPER};
inline constexpr double SHM_VEL_LIMIT[MOTOR_NUM] = {VEL_LIMIT_LEFT_SHOULDER_PITCH, VEL_LIMIT_LEFT_SHOULDER_ROLL, VEL_LIMIT_LEFT_SHOULDER_YAW, VEL_LIMIT_LEFT_ELBOW_PITCH, VEL_LIMIT_LEFT_WRIST_ROLL, VEL_LIMIT_LEFT_WRIST_YAW, VEL_LIMIT_LEFT_WRIST_PITCH, VEL_LIMIT_LEFT_GRIPPER, VEL_LIMIT_RIGHT_SHOULDER_PITCH, VEL_LIMIT_RIGHT_SHOULDER_ROLL, VEL_LIMIT_RIGHT_SHOULDER_YAW, VEL_LIMIT_RIGHT_ELBOW_PITCH, VEL_LIMIT_RIGHT_WRIST_ROLL, VEL_LIMIT_RIGHT_WRIST_YAW, VEL_LIMIT_RIGHT_WRIST_PITCH, VEL_LIMIT_RIGHT_GRIPPER};
inline constexpr double SHM_TORQUE_LIMIT[MOTOR_NUM] = {TORQUE_LIMIT_LEFT_SHOULDER_PITCH, TORQUE_LIMIT_LEFT_SHOULDER_ROLL, TORQUE_LIMIT_LEFT_SHOULDER_YAW, TORQUE_LIMIT_LEFT_ELBOW_PITCH, TORQUE_LIMIT_LEFT_WRIST_ROLL, TORQUE_LIMIT_LEFT_WRIST_YAW, TORQUE_LIMIT_LEFT_WRIST_PITCH, TORQUE_LIMIT_LEFT_GRIPPER, TORQUE_LIMIT_RIGHT_SHOULDER_PITCH, TORQUE_LIMIT_RIGHT_SHOULDER_ROLL, TORQUE_LIMIT_RIGHT_SHOULDER_YAW, TORQUE_LIMIT_RIGHT_ELBOW_PITCH, TORQUE_LIMIT_RIGHT_WRIST_ROLL, TORQUE_LIMIT_RIGHT_WRIST_YAW, TORQUE_LIMIT_RIGHT_WRIST_PITCH, TORQUE_LIMIT_RIGHT_GRIPPER};
inline constexpr double SHM_PP_VEL_MAX[MOTOR_NUM] = {PP_VEL_MAX_LEFT_SHOULDER_PITCH, PP_VEL_MAX_LEFT_SHOULDER_ROLL, PP_VEL_MAX_LEFT_SHOULDER_YAW, PP_VEL_MAX_LEFT_ELBOW_PITCH, PP_VEL_MAX_LEFT_WRIST_ROLL, PP_VEL_MAX_LEFT_WRIST_YAW, PP_VEL_MAX_LEFT_WRIST_PITCH, PP_VEL_MAX_LEFT_GRIPPER, PP_VEL_MAX_RIGHT_SHOULDER_PITCH, PP_VEL_MAX_RIGHT_SHOULDER_ROLL, PP_VEL_MAX_RIGHT_SHOULDER_YAW, PP_VEL_MAX_RIGHT_ELBOW_PITCH, PP_VEL_MAX_RIGHT_WRIST_ROLL, PP_VEL_MAX_RIGHT_WRIST_YAW, PP_VEL_MAX_RIGHT_WRIST_PITCH, PP_VEL_MAX_RIGHT_GRIPPER};
inline constexpr double SHM_PP_ACC[MOTOR_NUM] = {PP_ACC_LEFT_SHOULDER_PITCH, PP_ACC_LEFT_SHOULDER_ROLL, PP_ACC_LEFT_SHOULDER_YAW, PP_ACC_LEFT_ELBOW_PITCH, PP_ACC_LEFT_WRIST_ROLL, PP_ACC_LEFT_WRIST_YAW, PP_ACC_LEFT_WRIST_PITCH, PP_ACC_LEFT_GRIPPER, PP_ACC_RIGHT_SHOULDER_PITCH, PP_ACC_RIGHT_SHOULDER_ROLL, PP_ACC_RIGHT_SHOULDER_YAW, PP_ACC_RIGHT_ELBOW_PITCH, PP_ACC_RIGHT_WRIST_ROLL, PP_ACC_RIGHT_WRIST_YAW, PP_ACC_RIGHT_WRIST_PITCH, PP_ACC_RIGHT_GRIPPER};
inline constexpr double SHM_PP_CURRENT_LIMIT[MOTOR_NUM] = {PP_CURRENT_LIMIT_LEFT_SHOULDER_PITCH, PP_CURRENT_LIMIT_LEFT_SHOULDER_ROLL, PP_CURRENT_LIMIT_LEFT_SHOULDER_YAW, PP_CURRENT_LIMIT_LEFT_ELBOW_PITCH, PP_CURRENT_LIMIT_LEFT_WRIST_ROLL, PP_CURRENT_LIMIT_LEFT_WRIST_YAW, PP_CURRENT_LIMIT_LEFT_WRIST_PITCH, PP_CURRENT_LIMIT_LEFT_GRIPPER, PP_CURRENT_LIMIT_RIGHT_SHOULDER_PITCH, PP_CURRENT_LIMIT_RIGHT_SHOULDER_ROLL, PP_CURRENT_LIMIT_RIGHT_SHOULDER_YAW, PP_CURRENT_LIMIT_RIGHT_ELBOW_PITCH, PP_CURRENT_LIMIT_RIGHT_WRIST_ROLL, PP_CURRENT_LIMIT_RIGHT_WRIST_YAW, PP_CURRENT_LIMIT_RIGHT_WRIST_PITCH, PP_CURRENT_LIMIT_RIGHT_GRIPPER};
inline constexpr int SHM_CAN_IF_INDEX[MOTOR_NUM] = {CAN_IF_INDEX_LEFT_SHOULDER_PITCH, CAN_IF_INDEX_LEFT_SHOULDER_ROLL, CAN_IF_INDEX_LEFT_SHOULDER_YAW, CAN_IF_INDEX_LEFT_ELBOW_PITCH, CAN_IF_INDEX_LEFT_WRIST_ROLL, CAN_IF_INDEX_LEFT_WRIST_YAW, CAN_IF_INDEX_LEFT_WRIST_PITCH, CAN_IF_INDEX_LEFT_GRIPPER, CAN_IF_INDEX_RIGHT_SHOULDER_PITCH, CAN_IF_INDEX_RIGHT_SHOULDER_ROLL, CAN_IF_INDEX_RIGHT_SHOULDER_YAW, CAN_IF_INDEX_RIGHT_ELBOW_PITCH, CAN_IF_INDEX_RIGHT_WRIST_ROLL, CAN_IF_INDEX_RIGHT_WRIST_YAW, CAN_IF_INDEX_RIGHT_WRIST_PITCH, CAN_IF_INDEX_RIGHT_GRIPPER};

// shm 인덱스 -> 해당 타입의 모터 객체. 타입이 달라 배열로 못 다루므로 switch 로 골라 f 에 넘긴다.
template <class F>
inline void with_motor(Motor_con& motors, int shm_index, F&& f) {
    switch (shm_index) {
        case SHM_INDEX_LEFT_SHOULDER_PITCH:  f(std::get<RS02_Vec>(motors)[VEC_INDEX_LEFT_SHOULDER_PITCH]); return;
        case SHM_INDEX_LEFT_SHOULDER_ROLL:   f(std::get<RS02_Vec>(motors)[VEC_INDEX_LEFT_SHOULDER_ROLL]); return;
        case SHM_INDEX_LEFT_SHOULDER_YAW:    f(std::get<RS00_Vec>(motors)[VEC_INDEX_LEFT_SHOULDER_YAW]); return;
        case SHM_INDEX_LEFT_ELBOW_PITCH:     f(std::get<RS00_Vec>(motors)[VEC_INDEX_LEFT_ELBOW_PITCH]); return;
        case SHM_INDEX_LEFT_WRIST_ROLL:      f(std::get<EL05_Vec>(motors)[VEC_INDEX_LEFT_WRIST_ROLL]); return;
        case SHM_INDEX_LEFT_WRIST_YAW:       f(std::get<EL05_Vec>(motors)[VEC_INDEX_LEFT_WRIST_YAW]); return;
        case SHM_INDEX_LEFT_WRIST_PITCH:     f(std::get<EL05_Vec>(motors)[VEC_INDEX_LEFT_WRIST_PITCH]); return;
        case SHM_INDEX_LEFT_GRIPPER:         f(std::get<EL05_Vec>(motors)[VEC_INDEX_LEFT_GRIPPER]); return;
        case SHM_INDEX_RIGHT_SHOULDER_PITCH: f(std::get<RS02_Vec>(motors)[VEC_INDEX_RIGHT_SHOULDER_PITCH]); return;
        case SHM_INDEX_RIGHT_SHOULDER_ROLL:  f(std::get<RS02_Vec>(motors)[VEC_INDEX_RIGHT_SHOULDER_ROLL]); return;
        case SHM_INDEX_RIGHT_SHOULDER_YAW:   f(std::get<RS00_Vec>(motors)[VEC_INDEX_RIGHT_SHOULDER_YAW]); return;
        case SHM_INDEX_RIGHT_ELBOW_PITCH:    f(std::get<RS00_Vec>(motors)[VEC_INDEX_RIGHT_ELBOW_PITCH]); return;
        case SHM_INDEX_RIGHT_WRIST_ROLL:     f(std::get<EL05_Vec>(motors)[VEC_INDEX_RIGHT_WRIST_ROLL]); return;
        case SHM_INDEX_RIGHT_WRIST_YAW:      f(std::get<EL05_Vec>(motors)[VEC_INDEX_RIGHT_WRIST_YAW]); return;
        case SHM_INDEX_RIGHT_WRIST_PITCH:    f(std::get<EL05_Vec>(motors)[VEC_INDEX_RIGHT_WRIST_PITCH]); return;
        case SHM_INDEX_RIGHT_GRIPPER:        f(std::get<EL05_Vec>(motors)[VEC_INDEX_RIGHT_GRIPPER]); return;
        default: return;
    }
}

// 초기화용: shm 인덱스에 해당하는 타입의 deque 에 모터를 추가한다. (호출 순서 == VEC_INDEX 순서여야 하므로 shm 순서로 호출할 것)
inline void emplace_motor(Motor_con& motors, int shm_index, int can_socket, int can_id, double pos_offset_mech) {
    switch (shm_index) {
        case SHM_INDEX_LEFT_SHOULDER_PITCH:  std::get<RS02_Vec>(motors).emplace_back(can_socket, can_id, pos_offset_mech); return;
        case SHM_INDEX_LEFT_SHOULDER_ROLL:   std::get<RS02_Vec>(motors).emplace_back(can_socket, can_id, pos_offset_mech); return;
        case SHM_INDEX_LEFT_SHOULDER_YAW:    std::get<RS00_Vec>(motors).emplace_back(can_socket, can_id, pos_offset_mech); return;
        case SHM_INDEX_LEFT_ELBOW_PITCH:     std::get<RS00_Vec>(motors).emplace_back(can_socket, can_id, pos_offset_mech); return;
        case SHM_INDEX_LEFT_WRIST_ROLL:      std::get<EL05_Vec>(motors).emplace_back(can_socket, can_id, pos_offset_mech); return;
        case SHM_INDEX_LEFT_WRIST_YAW:       std::get<EL05_Vec>(motors).emplace_back(can_socket, can_id, pos_offset_mech); return;
        case SHM_INDEX_LEFT_WRIST_PITCH:     std::get<EL05_Vec>(motors).emplace_back(can_socket, can_id, pos_offset_mech); return;
        case SHM_INDEX_LEFT_GRIPPER:         std::get<EL05_Vec>(motors).emplace_back(can_socket, can_id, pos_offset_mech); return;
        case SHM_INDEX_RIGHT_SHOULDER_PITCH: std::get<RS02_Vec>(motors).emplace_back(can_socket, can_id, pos_offset_mech); return;
        case SHM_INDEX_RIGHT_SHOULDER_ROLL:  std::get<RS02_Vec>(motors).emplace_back(can_socket, can_id, pos_offset_mech); return;
        case SHM_INDEX_RIGHT_SHOULDER_YAW:   std::get<RS00_Vec>(motors).emplace_back(can_socket, can_id, pos_offset_mech); return;
        case SHM_INDEX_RIGHT_ELBOW_PITCH:    std::get<RS00_Vec>(motors).emplace_back(can_socket, can_id, pos_offset_mech); return;
        case SHM_INDEX_RIGHT_WRIST_ROLL:     std::get<EL05_Vec>(motors).emplace_back(can_socket, can_id, pos_offset_mech); return;
        case SHM_INDEX_RIGHT_WRIST_YAW:      std::get<EL05_Vec>(motors).emplace_back(can_socket, can_id, pos_offset_mech); return;
        case SHM_INDEX_RIGHT_WRIST_PITCH:    std::get<EL05_Vec>(motors).emplace_back(can_socket, can_id, pos_offset_mech); return;
        case SHM_INDEX_RIGHT_GRIPPER:        std::get<EL05_Vec>(motors).emplace_back(can_socket, can_id, pos_offset_mech); return;
        default: return;
    }
}

// SHM_INDEX -> CAN ID. 표에 없는 인덱스는 0 (유효하지 않은 ID).
constexpr uint32_t shm_index_to_can_id(int shm_index) {
    switch (shm_index) {
        case SHM_INDEX_LEFT_SHOULDER_PITCH:  return CAN_ID_LEFT_SHOULDER_PITCH;
        case SHM_INDEX_LEFT_SHOULDER_ROLL:   return CAN_ID_LEFT_SHOULDER_ROLL;
        case SHM_INDEX_LEFT_SHOULDER_YAW:    return CAN_ID_LEFT_SHOULDER_YAW;
        case SHM_INDEX_LEFT_ELBOW_PITCH:     return CAN_ID_LEFT_ELBOW_PITCH;
        case SHM_INDEX_LEFT_WRIST_ROLL:      return CAN_ID_LEFT_WRIST_ROLL;
        case SHM_INDEX_LEFT_WRIST_YAW:       return CAN_ID_LEFT_WRIST_YAW;
        case SHM_INDEX_LEFT_WRIST_PITCH:     return CAN_ID_LEFT_WRIST_PITCH;
        case SHM_INDEX_LEFT_GRIPPER:         return CAN_ID_LEFT_GRIPPER;
        case SHM_INDEX_RIGHT_SHOULDER_PITCH: return CAN_ID_RIGHT_SHOULDER_PITCH;
        case SHM_INDEX_RIGHT_SHOULDER_ROLL:  return CAN_ID_RIGHT_SHOULDER_ROLL;
        case SHM_INDEX_RIGHT_SHOULDER_YAW:   return CAN_ID_RIGHT_SHOULDER_YAW;
        case SHM_INDEX_RIGHT_ELBOW_PITCH:    return CAN_ID_RIGHT_ELBOW_PITCH;
        case SHM_INDEX_RIGHT_WRIST_ROLL:     return CAN_ID_RIGHT_WRIST_ROLL;
        case SHM_INDEX_RIGHT_WRIST_YAW:      return CAN_ID_RIGHT_WRIST_YAW;
        case SHM_INDEX_RIGHT_WRIST_PITCH:    return CAN_ID_RIGHT_WRIST_PITCH;
        case SHM_INDEX_RIGHT_GRIPPER:        return CAN_ID_RIGHT_GRIPPER;
        default: return 0;
    }
}

constexpr bool all_shm_index_mapped() {
    for (int i = 0; i < MOTOR_NUM; ++i) if (shm_index_to_can_id(i) == 0) return false;
    return true;
}
static_assert(all_shm_index_mapped(), "shm_index_to_can_id 가 0..MOTOR_NUM-1 을 모두 덮지 못함");

#endif  // AREUMII_CONFIG_H
