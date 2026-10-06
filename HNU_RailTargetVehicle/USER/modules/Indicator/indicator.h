#ifndef RAIL_TARGET_INDICATOR_H
#define RAIL_TARGET_INDICATOR_H

#include <stdint.h>

void Indicator_Init(void);
void Indicator_Process(void);
void Indicator_SetBluetoothConnected(uint8_t connected);
void Indicator_FaultLatch(void);

#endif
