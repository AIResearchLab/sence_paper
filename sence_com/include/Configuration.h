#define BAUD_RATE 115200
#define UPPER_BYTE(b) (b >> 8) //defines byte structure
#define LOWER_BYTE(b) (b & 0xff)
#define INT_JOIN_BYTE(u, l) (u << 8) | l
#define HeaderValue 60000
#define M_PI    3.14159265358979323846  /* pi */

#define GOAL_SPEED_RPM_TICK 0.114
#define SPEED_FEEDBACK_TICK 0.11
#define GOAL_POSITION_DYNA 0.088

enum JOINT_DEST {
    D_1 = 20,
    D_2,
    D_3,
    D_4,
    D_5,
    D_6,
    D_7,
    D_8,
    D_9,
    D_10,
    D_11,
    D_12};

enum INTERFACE_CMD_DYNAMIXEL {
    TARGET_POSITION = 40,
    PRESENT_POSITION,
    TARGET_VELOCITY,
    PRESENT_VELOCITY,
    PRESENT_TEMP};

enum MSG_FRAGMENT {
    _DEST,
    _CMD,
    _UPPER_BYTE,
    _LOWER_BYTE};