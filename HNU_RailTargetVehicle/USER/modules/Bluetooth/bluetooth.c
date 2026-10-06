#include "Bluetooth/bluetooth.h"

#include <string.h>

#include "Indicator/indicator.h"
#include "TargetMotor/target_motor.h"
#include "main.h"
#include "ranging_task.h"
#include "usart.h"

#define BT_SOF_1             0xAAU
#define BT_SOF_2             0x55U
#define BT_VERSION           0x01U
#define BT_MAX_PAYLOAD       40U
#define BT_RX_RING_SIZE      256U
#define BT_LINK_TIMEOUT_MS   600U
#define BT_REMOTE_TIMEOUT_MS 400U

#define BT_STATUS_OK           0U
#define BT_STATUS_BAD_COMMAND  1U
#define BT_STATUS_BAD_PAYLOAD  2U
#define BT_STATUS_ESTOP_ACTIVE 3U
#define BT_STATUS_WRONG_MODE   4U
#define BT_STATUS_MOTOR_MOVING 5U

#define BT_SPEED_CONFIG_TRACK_STOP_RPM 50U
#define BT_SPEED_CONFIG_ARMOR_STOP_RPM 20U

volatile bt_test_state_t g_bt_test_state;

static uint8_t rx_interrupt_byte;
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static uint8_t rx_ring[BT_RX_RING_SIZE];
static uint8_t parser_state;
static uint8_t parser_buffer[4U + BT_MAX_PAYLOAD + 2U];
static uint8_t parser_index;
static uint8_t parser_expected;
static uint32_t last_valid_frame_tick;
static uint32_t last_remote_command_tick;

static uint16_t crc16_modbus(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    for (uint16_t i = 0U; i < length; ++i)
    {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; ++bit)
        {
            crc = (crc & 1U) ? (uint16_t)((crc >> 1U) ^ 0xA001U) :
                               (uint16_t)(crc >> 1U);
        }
    }
    return crc;
}

static int16_t read_i16_le(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

static uint16_t read_u16_le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static uint16_t abs_i16(int16_t value)
{
    return (value < 0) ? (uint16_t)(-(int32_t)value) : (uint16_t)value;
}

static void write_u16_le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)(value >> 8U);
}

static int16_t clamp_control(int16_t value)
{
    if (value > 1000) return 1000;
    if (value < -1000) return -1000;
    return value;
}

static void send_frame(uint8_t command, uint8_t sequence,
                       const uint8_t *payload, uint8_t payload_length)
{
    uint8_t frame[8U + BT_MAX_PAYLOAD];
    if (payload_length > BT_MAX_PAYLOAD) return;

    frame[0] = BT_SOF_1;
    frame[1] = BT_SOF_2;
    frame[2] = BT_VERSION;
    frame[3] = command;
    frame[4] = sequence;
    frame[5] = payload_length;
    if ((payload != NULL) && (payload_length > 0U))
    {
        memcpy(&frame[6], payload, payload_length);
    }

    const uint16_t crc = crc16_modbus(&frame[2], (uint16_t)(4U + payload_length));
    write_u16_le(&frame[6U + payload_length], crc);
    (void)HAL_UART_Transmit(&huart1, frame, (uint16_t)(8U + payload_length), 20U);
}

static void send_ack(uint8_t command, uint8_t sequence, uint8_t status)
{
    const uint8_t payload[2] = {command, status};
    send_frame(BT_CMD_ACK, sequence, payload, sizeof(payload));
}

static void send_status(uint8_t sequence)
{
    uint8_t payload[39] = {0};
    uint8_t ranging_flags = 0U;

    payload[0] = g_bt_test_state.mode;
    payload[1] = g_bt_test_state.emergency_stop;
    payload[2] = g_bt_test_state.link_online;
    payload[3] = (uint8_t)((g_target_motor_test_state.track_online ? 0x01U : 0U) |
                           (g_target_motor_test_state.armor_online ? 0x02U : 0U));
    write_u16_le(&payload[4], (uint16_t)g_target_motor_test_state.track_target_rpm);
    write_u16_le(&payload[6], (uint16_t)g_target_motor_test_state.track_speed_rpm);
    write_u16_le(&payload[8], (uint16_t)g_target_motor_test_state.track_current);
    write_u16_le(&payload[10], (uint16_t)g_target_motor_test_state.armor_target_rpm);
    write_u16_le(&payload[12], (uint16_t)g_target_motor_test_state.armor_speed_rpm);
    write_u16_le(&payload[14], (uint16_t)g_target_motor_test_state.armor_current);
    write_u16_le(&payload[16], g_target_motor_test_state.track_ecd);
    write_u16_le(&payload[18], g_target_motor_test_state.armor_ecd);
    /* Identifies bytes 8/14 as CAN feedback current, not legacy PID output. */
    payload[20] = 0x43U;
    payload[21] = 0x49U;
    payload[22] = g_target_motor_test_state.init_ok;

    if (g_ranging_state.init_ok) ranging_flags |= 0x01U;
    if (g_ranging_state.left_valid) ranging_flags |= 0x02U;
    if (g_ranging_state.right_valid) ranging_flags |= 0x04U;
    if (g_ranging_state.left_endpoint) ranging_flags |= 0x08U;
    if (g_ranging_state.right_endpoint) ranging_flags |= 0x10U;
    if (g_ranging_state.left_alive) ranging_flags |= 0x20U;
    if (g_ranging_state.right_alive) ranging_flags |= 0x40U;
    payload[23] = ranging_flags;
    write_u16_le(&payload[24], g_ranging_state.left_mm);
    write_u16_le(&payload[26], g_ranging_state.right_mm);
    payload[28] = (uint8_t)g_target_motor_test_state.auto_move_direction;
    write_u16_le(&payload[29], g_bt_test_state.auto_track_rpm);
    write_u16_le(&payload[31], g_bt_test_state.auto_armor_rpm);
    payload[33] = g_ranging_state.left_range_status;
    payload[34] = g_ranging_state.right_range_status;
    write_u16_le(&payload[35], g_bt_test_state.remote_track_full_scale_rpm);
    write_u16_le(&payload[37], g_bt_test_state.remote_armor_full_scale_rpm);

    send_frame(BT_CMD_STATUS, sequence, payload, sizeof(payload));
}

static void stop_motion_command(void)
{
    g_bt_test_state.move_command = 0;
    g_bt_test_state.rotate_command = 0;
}

static void handle_frame(uint8_t command, uint8_t sequence,
                         const uint8_t *payload, uint8_t payload_length)
{
    uint8_t status = BT_STATUS_OK;
    const uint32_t now = HAL_GetTick();

    g_bt_test_state.valid_frames++;
    g_bt_test_state.link_online = 1U;
    g_bt_test_state.last_command = command;
    g_bt_test_state.last_sequence = sequence;
    last_valid_frame_tick = now;

    switch (command)
    {
        case BT_CMD_PING:
        case BT_CMD_HEARTBEAT:
            if (payload_length != 0U) status = BT_STATUS_BAD_PAYLOAD;
            break;

        case BT_CMD_SET_MODE:
            if ((payload_length != 1U) || (payload[0] > BT_MODE_AUTO_BOTH))
            {
                status = BT_STATUS_BAD_PAYLOAD;
            }
            else if (g_bt_test_state.emergency_stop && (payload[0] != BT_MODE_STOP))
            {
                status = BT_STATUS_ESTOP_ACTIVE;
            }
            else
            {
                g_bt_test_state.mode = payload[0];
                stop_motion_command();
            }
            break;

        case BT_CMD_SET_REMOTE:
            if (payload_length != 4U)
            {
                status = BT_STATUS_BAD_PAYLOAD;
            }
            else if (g_bt_test_state.emergency_stop)
            {
                status = BT_STATUS_ESTOP_ACTIVE;
            }
            else if (g_bt_test_state.mode != BT_MODE_REMOTE)
            {
                status = BT_STATUS_WRONG_MODE;
            }
            else
            {
                g_bt_test_state.move_command = clamp_control(read_i16_le(&payload[0]));
                g_bt_test_state.rotate_command = clamp_control(read_i16_le(&payload[2]));
                last_remote_command_tick = now;
            }
            break;

        case BT_CMD_ESTOP:
            if (payload_length != 0U)
            {
                status = BT_STATUS_BAD_PAYLOAD;
            }
            else
            {
                g_bt_test_state.emergency_stop = 1U;
                g_bt_test_state.mode = BT_MODE_STOP;
                stop_motion_command();
            }
            break;

        case BT_CMD_CLEAR_ESTOP:
            if (payload_length != 0U)
            {
                status = BT_STATUS_BAD_PAYLOAD;
            }
            else if (g_bt_test_state.emergency_stop)
            {
                g_bt_test_state.emergency_stop = 0U;
                g_bt_test_state.mode = BT_MODE_STOP;
                stop_motion_command();
            }
            /* If no emergency stop is active, clearing it is a harmless no-op. */
            break;

        case BT_CMD_SET_AUTO_SPEED:
            if (payload_length != 4U)
            {
                status = BT_STATUS_BAD_PAYLOAD;
            }
            else if (g_bt_test_state.emergency_stop)
            {
                status = BT_STATUS_ESTOP_ACTIVE;
            }
            else if ((g_bt_test_state.mode != BT_MODE_STOP) &&
                     (g_bt_test_state.mode != BT_MODE_REMOTE))
            {
                status = BT_STATUS_WRONG_MODE;
            }
            else if ((!g_target_motor_test_state.track_online) ||
                     (!g_target_motor_test_state.armor_online) ||
                     (abs_i16(g_target_motor_test_state.track_speed_rpm) >
                      BT_SPEED_CONFIG_TRACK_STOP_RPM) ||
                     (abs_i16(g_target_motor_test_state.armor_speed_rpm) >
                      BT_SPEED_CONFIG_ARMOR_STOP_RPM))
            {
                status = BT_STATUS_MOTOR_MOVING;
            }
            else
            {
                const uint16_t track_rpm = read_u16_le(&payload[0]);
                const uint16_t armor_rpm = read_u16_le(&payload[2]);
                if ((track_rpm < BT_AUTO_TRACK_MIN_RPM) ||
                    (track_rpm > BT_AUTO_TRACK_MAX_RPM) ||
                    (armor_rpm < BT_AUTO_ARMOR_MIN_RPM) ||
                    (armor_rpm > BT_AUTO_ARMOR_MAX_RPM))
                {
                    status = BT_STATUS_BAD_PAYLOAD;
                }
                else
                {
                    g_bt_test_state.auto_track_rpm = track_rpm;
                    g_bt_test_state.auto_armor_rpm = armor_rpm;
                }
            }
            break;

        case BT_CMD_SET_REMOTE_MAPPING:
            /* Two bytes keeps older track-only clients working; four updates both axes. */
            if ((payload_length != 2U) && (payload_length != 4U))
            {
                status = BT_STATUS_BAD_PAYLOAD;
            }
            else if (g_bt_test_state.emergency_stop)
            {
                status = BT_STATUS_ESTOP_ACTIVE;
            }
            else if ((g_bt_test_state.mode != BT_MODE_STOP) &&
                     (g_bt_test_state.mode != BT_MODE_REMOTE))
            {
                status = BT_STATUS_WRONG_MODE;
            }
            else if ((!g_target_motor_test_state.track_online) ||
                     (abs_i16(g_target_motor_test_state.track_speed_rpm) >
                      BT_SPEED_CONFIG_TRACK_STOP_RPM) ||
                     ((payload_length == 4U) &&
                      ((!g_target_motor_test_state.armor_online) ||
                       (abs_i16(g_target_motor_test_state.armor_speed_rpm) >
                        BT_SPEED_CONFIG_ARMOR_STOP_RPM))))
            {
                status = BT_STATUS_MOTOR_MOVING;
            }
            else
            {
                const uint16_t track_rpm = read_u16_le(payload);
                const uint16_t armor_rpm = (payload_length == 4U) ?
                    read_u16_le(&payload[2]) :
                    g_bt_test_state.remote_armor_full_scale_rpm;
                if ((track_rpm < BT_REMOTE_TRACK_MIN_RPM) ||
                    (track_rpm > BT_REMOTE_TRACK_MAX_RPM) ||
                    (armor_rpm < BT_REMOTE_ARMOR_MIN_RPM) ||
                    (armor_rpm > BT_REMOTE_ARMOR_MAX_RPM))
                {
                    status = BT_STATUS_BAD_PAYLOAD;
                }
                else
                {
                    g_bt_test_state.remote_track_full_scale_rpm = track_rpm;
                    g_bt_test_state.remote_armor_full_scale_rpm = armor_rpm;
                }
            }
            break;

        default:
            status = BT_STATUS_BAD_COMMAND;
            break;
    }

    send_ack(command, sequence, status);
    if ((command == BT_CMD_PING) && (status == BT_STATUS_OK))
    {
        send_frame(BT_CMD_PONG, sequence, NULL, 0U);
    }
    send_status(sequence);
}

static void parse_byte(uint8_t byte)
{
    if (parser_state == 0U)
    {
        if (byte == BT_SOF_1) parser_state = 1U;
        return;
    }
    if (parser_state == 1U)
    {
        if (byte == BT_SOF_2)
        {
            parser_state = 2U;
            parser_index = 0U;
            parser_expected = 0U;
        }
        else if (byte != BT_SOF_1)
        {
            parser_state = 0U;
        }
        return;
    }

    parser_buffer[parser_index++] = byte;
    if (parser_index == 4U)
    {
        if (parser_buffer[3] > BT_MAX_PAYLOAD)
        {
            g_bt_test_state.framing_errors++;
            parser_state = 0U;
            return;
        }
        parser_expected = (uint8_t)(4U + parser_buffer[3] + 2U);
    }

    if ((parser_expected != 0U) && (parser_index >= parser_expected))
    {
        const uint8_t length = parser_buffer[3];
        const uint16_t received_crc = (uint16_t)parser_buffer[4U + length] |
                                      ((uint16_t)parser_buffer[5U + length] << 8U);
        const uint16_t calculated_crc = crc16_modbus(parser_buffer, (uint16_t)(4U + length));
        if ((parser_buffer[0] == BT_VERSION) && (received_crc == calculated_crc))
        {
            handle_frame(parser_buffer[1], parser_buffer[2], &parser_buffer[4], length);
        }
        else
        {
            g_bt_test_state.crc_errors++;
        }
        parser_state = 0U;
    }
}

void Bluetooth_Init(void)
{
    memset((void *)&g_bt_test_state, 0, sizeof(g_bt_test_state));
    g_bt_test_state.mode = BT_MODE_STOP;
    g_bt_test_state.auto_track_rpm = BT_AUTO_TRACK_DEFAULT_RPM;
    g_bt_test_state.auto_armor_rpm = BT_AUTO_ARMOR_DEFAULT_RPM;
    g_bt_test_state.remote_track_full_scale_rpm = BT_REMOTE_TRACK_DEFAULT_RPM;
    g_bt_test_state.remote_armor_full_scale_rpm = BT_REMOTE_ARMOR_DEFAULT_RPM;
    rx_head = 0U;
    rx_tail = 0U;
    parser_state = 0U;
    last_valid_frame_tick = HAL_GetTick();
    last_remote_command_tick = HAL_GetTick();
    Indicator_SetBluetoothConnected(0U);
    (void)HAL_UART_Receive_IT(&huart1, &rx_interrupt_byte, 1U);
}

void Bluetooth_Process(void)
{
    while (rx_tail != rx_head)
    {
        const uint8_t byte = rx_ring[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1U) % BT_RX_RING_SIZE);
        parse_byte(byte);
    }

    const uint32_t now = HAL_GetTick();
    if (g_bt_test_state.link_online && ((now - last_valid_frame_tick) > BT_LINK_TIMEOUT_MS))
    {
        g_bt_test_state.link_online = 0U;
        g_bt_test_state.mode = BT_MODE_STOP;
        stop_motion_command();
    }
    if ((g_bt_test_state.mode == BT_MODE_REMOTE) &&
        ((now - last_remote_command_tick) > BT_REMOTE_TIMEOUT_MS))
    {
        stop_motion_command();
    }
    Indicator_SetBluetoothConnected(g_bt_test_state.link_online);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        const uint16_t next = (uint16_t)((rx_head + 1U) % BT_RX_RING_SIZE);
        if (next != rx_tail)
        {
            rx_ring[rx_head] = rx_interrupt_byte;
            rx_head = next;
        }
        else
        {
            g_bt_test_state.framing_errors++;
        }
        (void)HAL_UART_Receive_IT(&huart1, &rx_interrupt_byte, 1U);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        __HAL_UART_CLEAR_OREFLAG(&huart1);
        (void)HAL_UART_Receive_IT(&huart1, &rx_interrupt_byte, 1U);
    }
}
