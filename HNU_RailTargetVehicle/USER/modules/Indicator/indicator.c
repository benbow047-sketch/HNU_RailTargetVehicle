#include "Indicator/indicator.h"

#include "main.h"

#define INDICATOR_BLINK_HALF_PERIOD_MS 500U

static uint8_t bluetooth_connected;
static uint8_t fault_latched;

void Indicator_Init(void)
{
    bluetooth_connected = 0U;
    fault_latched = 0U;
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

void Indicator_Process(void)
{
    GPIO_PinState state;

    if (fault_latched || bluetooth_connected)
    {
        state = GPIO_PIN_SET;
    }
    else
    {
        state = (((HAL_GetTick() / INDICATOR_BLINK_HALF_PERIOD_MS) & 1U) != 0U) ?
                GPIO_PIN_SET : GPIO_PIN_RESET;
    }

    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, state);
}

void Indicator_SetBluetoothConnected(uint8_t connected)
{
    bluetooth_connected = connected ? 1U : 0U;
}

void Indicator_FaultLatch(void)
{
    fault_latched = 1U;
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
}
