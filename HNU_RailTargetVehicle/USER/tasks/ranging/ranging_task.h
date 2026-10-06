#ifndef RAIL_TARGET_RANGING_TASK_H
#define RAIL_TARGET_RANGING_TASK_H

#include <stdint.h>

#define RANGING_DATA_TIMEOUT_MS     300U

typedef struct
{
    uint8_t init_ok;
    uint8_t left_valid;  /* Raw Hall input after active-low conversion. */
    uint8_t right_valid;
    uint8_t left_alive;
    uint8_t right_alive;
    uint8_t left_range_status;
    uint8_t right_range_status;
    uint8_t left_endpoint;
    uint8_t right_endpoint;
    uint16_t left_mm;    /* Reserved for protocol compatibility. */
    uint16_t right_mm;
    uint32_t left_update_tick;
    uint32_t right_update_tick;
    uint32_t left_error_count;
    uint32_t right_error_count;
} ranging_state_t;

extern volatile ranging_state_t g_ranging_state;

void RangingTaskEntry(void *argument);
uint8_t Ranging_AutomaticReady(uint32_t now);

#endif
