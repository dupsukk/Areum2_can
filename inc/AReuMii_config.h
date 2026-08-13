#define CAN_ID_LEFT_SHOULDER_PITCH   0x01
#define CAN_ID_LEFT_SHOULDER_ROLL    0x02
#define CAN_ID_LEFT_SHOULDER_YAW     0x03
#define CAN_ID_LEFT_ELBOW            0x04
#define CAN_ID_LEFT_WRIST_ROLL       0x05
#define CAN_ID_LEFT_WRIST_YAW        0x06
#define CAN_ID_LEFT_WRIST_PITCH      0x07
#define CAN_ID_LEFT_EE               0x08

#define CAN_ID_RIGHT_SHOULDER_PITCH  0x11
#define CAN_ID_RIGHT_SHOULDER_ROLL   0x12
#define CAN_ID_RIGHT_SHOULDER_YAW    0x13
#define CAN_ID_RIGHT_ELBOW           0x14
#define CAN_ID_RIGHT_WRIST_ROLL      0x15
#define CAN_ID_RIGHT_WRIST_YAW       0x16
#define CAN_ID_RIGHT_WRIST_PITCH     0x17
#define CAN_ID_RIGHT_EE              0x18

//shared memory index 
#define SHM_INDEX_LEFT_SHOULDER_PITCH     0
#define SHM_INDEX_LEFT_SHOULDER_ROLL    1
#define SHM_INDEX_LEFT_SHOULDER_YAW      2 
#define SHM_INDEX_LEFT_ELBOW_PITCH       3
#define SHM_INDEX_LEFT_WRIST_ROLL        4
#define SHM_INDEX_LEFT_WRIST_YAW         5 
#define SHM_INDEX_LEFT_WRIST_PITCH       6
#define SHM_INDEX_LEFT_GRIPPER           7

#define SHM_INDEX_RIGHT_SHOULDER_PITCH    8
#define SHM_INDEX_RIGHT_SHOULDER_ROLL   9
#define SHM_INDEX_RIGHT_SHOULDER_YAW     10
#define SHM_INDEX_RIGHT_ELBOW_PITCH      11
#define SHM_INDEX_RIGHT_WRIST_ROLL       12
#define SHM_INDEX_RIGHT_WRIST_YAW        13
#define SHM_INDEX_RIGHT_WRIST_PITCH      14
#define SHM_INDEX_RIGHT_GRIPPER          15


// Vector index 
#define VEC_INDEX_LEFT_SHOULDER_PITCH    0
#define VEC_INDEX_LEFT_SHOULDER_ROLL     1
#define VEC_INDEX_LEFT_SHOULDER_YAW      0
#define VEC_INDEX_LEFT_ELBOW_PITCH       1
#define VEC_INDEX_LEFT_WRIST_ROLL        0
#define VEC_INDEX_LEFT_WRIST_YAW         1
#define VEC_INDEX_LEFT_WRIST_PITCH       2
#define VEC_INDEX_LEFT_GRIPPER           3

#define VEC_INDEX_RIGHT_SHOULDER_PITCH   2
#define VEC_INDEX_RIGHT_SHOULDER_ROLL    3
#define VEC_INDEX_RIGHT_SHOULDER_YAW     2
#define VEC_INDEX_RIGHT_ELBOW_PITCH      3
#define VEC_INDEX_RIGHT_WRIST_ROLL       4
#define VEC_INDEX_RIGHT_WRIST_YAW        5
#define VEC_INDEX_RIGHT_WRIST_PITCH      6
#define VEC_INDEX_RIGHT_GRIPPER          7



//lEFT ARM LIMITS
constexpr double MAX_POS_LEFT_SHOULDER_PITCH =   1.0;
constexpr double MIN_POS_LEFT_SHOULDER_PITCH =  -1.0;
constexpr double MAX_POS_LEFT_SHOULDER_ROLL =    1.0;
constexpr double MIN_POS_LEFT_SHOULDER_ROLL =   -1.0;
constexpr double MAX_POS_LEFT_SHOULDER_YAW =     1.0;
constexpr double MIN_POS_LEFT_SHOULDER_YAW =    -1.0;
constexpr double MAX_POS_LEFT_ELBOW =            1.0;
constexpr double MIN_POS_LEFT_ELBOW =           -1.0;
constexpr double MAX_POS_LEFT_WRIST_ROLL =       1.0;
constexpr double MIN_POS_LEFT_WRIST_ROLL =      -1.0;
constexpr double MAX_POS_LEFT_WRIST_PITCH =      1.0;
constexpr double MIN_POS_LEFT_WRIST_PITCH =     -1.0;
constexpr double MAX_POS_LEFT_WRIST_YAW =        1.0;
constexpr double MIN_POS_LEFT_WRIST_YAW =       -1.0;
constexpr double MAX_POS_LEFT_EE =               1.0;
constexpr double MIN_POS_LEFT_EE =              -1.0;

//RIGHT ARM LIMITS 
constexpr double MAX_POS_RIGHT_SHOULDER_PITCH =   1.0;
constexpr double MIN_POS_RIGHT_SHOULDER_PITCH =  -1.0;
constexpr double MAX_POS_RIGHT_SHOULDER_ROLL =    1.0;
constexpr double MIN_POS_RIGHT_SHOULDER_ROLL =   -1.0;
constexpr double MAX_POS_RIGHT_SHOULDER_YAW =     1.0;
constexpr double MIN_POS_RIGHT_SHOULDER_YAW =    -1.0;
constexpr double MAX_POS_RIGHT_ELBOW =            1.0;
constexpr double MIN_POS_RIGHT_ELBOW =           -1.0;
constexpr double MAX_POS_RIGHT_WRIST_ROLL =       1.0;
constexpr double MIN_POS_RIGHT_WRIST_ROLL =      -1.0;
constexpr double MAX_POS_RIGHT_WRIST_PITCH =      1.0;
constexpr double MIN_POS_RIGHT_WRIST_PITCH =     -1.0;
constexpr double MAX_POS_RIGHT_WRIST_YAW =        1.0;
constexpr double MIN_POS_RIGHT_WRIST_YAW =       -1.0;
constexpr double MAX_POS_RIGHT_EE =               1.0;
constexpr double MIN_POS_RIGHT_EE =              -1.0;


