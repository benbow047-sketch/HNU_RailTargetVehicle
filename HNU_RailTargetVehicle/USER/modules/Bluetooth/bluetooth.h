#ifndef RAIL_TARGET_BLUETOOTH_H
#define RAIL_TARGET_BLUETOOTH_H

#include <stdint.h>

#define BT_AUTO_TRACK_MIN_RPM    200U
#define BT_AUTO_TRACK_MAX_RPM   4536U
#define BT_AUTO_TRACK_DEFAULT_RPM 1500U
#define BT_AUTO_ARMOR_MIN_RPM     50U
#define BT_AUTO_ARMOR_MAX_RPM    400U
#define BT_AUTO_ARMOR_DEFAULT_RPM 200U
#define BT_REMOTE_TRACK_MIN_RPM    200U
#define BT_REMOTE_TRACK_MAX_RPM   4536U
#define BT_REMOTE_TRACK_DEFAULT_RPM 1500U
#define BT_REMOTE_ARMOR_MIN_RPM      20U
#define BT_REMOTE_ARMOR_MAX_RPM     400U
#define BT_REMOTE_ARMOR_DEFAULT_RPM 200U

typedef enum
{
    BT_MODE_STOP = 0,
    BT_MODE_REMOTE = 1,
    BT_MODE_AUTO_ROTATE = 2,
    BT_MODE_AUTO_MOVE = 3,
    BT_MODE_AUTO_BOTH = 4
} bt_mode_t;

typedef enum
{
    BT_CMD_PING = 0x01,
    BT_CMD_HEARTBEAT = 0x02,
    BT_CMD_SET_MODE = 0x10,
    BT_CMD_SET_REMOTE = 0x11,
    BT_CMD_ESTOP = 0x12,
    BT_CMD_CLEAR_ESTOP = 0x13,
    BT_CMD_SET_AUTO_SPEED = 0x14,
    BT_CMD_SET_REMOTE_MAPPING = 0x15,
    BT_CMD_ACK = 0x80,
    BT_CMD_PONG = 0x81,
    BT_CMD_STATUS = 0x82
} bt_command_t;

typedef struct
{
    uint8_t mode;
    uint8_t emergency_stop;
    uint8_t link_online;
    uint8_t last_command;
    int16_t move_command;
    int16_t rotate_command;
    uint16_t auto_track_rpm;
    uint16_t auto_armor_rpm;
    uint16_t remote_track_full_scale_rpm;
    uint16_t remote_armor_full_scale_rpm;
    uint8_t last_sequence;
    uint32_t valid_frames;
    uint32_t crc_errors;
    uint32_t framing_errors;
} bt_test_state_t;

extern volatile bt_test_state_t g_bt_test_state;

void Bluetooth_Init(void);
void Bluetooth_Process(void);

#endif
