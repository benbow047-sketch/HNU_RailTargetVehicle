#include "ranging_task.h"

#include <string.h>

#include "cmsis_os2.h"
#include "main.h"

#define HALL_CONFIRM_SAMPLES  3U
#define HALL_SAMPLE_PERIOD_MS 5U

volatile ranging_state_t g_ranging_state;

static uint8_t left_active_count;
static uint8_t left_release_count;
static uint8_t right_active_count;
static uint8_t right_release_count;

static void update_hall_endpoint(volatile uint8_t *endpoint,
                                 uint8_t raw_active,
                                 uint8_t *active_count,
                                 uint8_t *release_count)
{
    if (raw_active)
    {
        *release_count = 0U;
        if (*active_count < HALL_CONFIRM_SAMPLES) (*active_count)++;
        if (*active_count >= HALL_CONFIRM_SAMPLES) *endpoint = 1U;
    }
    else
    {
        *active_count = 0U;
        if (*release_count < HALL_CONFIRM_SAMPLES) (*release_count)++;
        if (*release_count >= HALL_CONFIRM_SAMPLES) *endpoint = 0U;
    }
}

uint8_t Ranging_AutomaticReady(uint32_t now)
{
    return (uint8_t)(g_ranging_state.init_ok &&
                     g_ranging_state.left_alive &&
                     g_ranging_state.right_alive &&
                     ((now - g_ranging_state.left_update_tick) <=
                      RANGING_DATA_TIMEOUT_MS) &&
                     ((now - g_ranging_state.right_update_tick) <=
                      RANGING_DATA_TIMEOUT_MS));
}

void RangingTaskEntry(void *argument)
{
    (void)argument;
    memset((void *)&g_ranging_state, 0, sizeof(g_ranging_state));

    g_ranging_state.init_ok = 1U;
    g_ranging_state.left_alive = 1U;
    g_ranging_state.right_alive = 1U;

    for (;;)
    {
        const uint32_t now = HAL_GetTick();
        const uint8_t left_raw_active =
            (uint8_t)(HAL_GPIO_ReadPin(HALL_LEFT_GPIO_Port, HALL_LEFT_Pin) ==
                      GPIO_PIN_RESET);
        const uint8_t right_raw_active =
            (uint8_t)(HAL_GPIO_ReadPin(HALL_RIGHT_GPIO_Port, HALL_RIGHT_Pin) ==
                      GPIO_PIN_RESET);

        /* ES3144 module digital output is active low. */
        g_ranging_state.left_valid = left_raw_active;
        g_ranging_state.right_valid = right_raw_active;
        g_ranging_state.left_update_tick = now;
        g_ranging_state.right_update_tick = now;

        update_hall_endpoint(&g_ranging_state.left_endpoint,
                             left_raw_active,
                             &left_active_count, &left_release_count);
        update_hall_endpoint(&g_ranging_state.right_endpoint,
                             right_raw_active,
                             &right_active_count, &right_release_count);

        osDelay(HALL_SAMPLE_PERIOD_MS);
    }
}
