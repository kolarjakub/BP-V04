#ifndef _RPM_H
#define _RPM_H

#define RPM_LIMIT 70
#define RPM_THRESHOLD 10
#define RPM_PULSES 200	// LOWEST VALUE OF RPM, VALUES LESS THAN RPM_PULSES ARE SET TO ZERO
//#define RPM_MIN_MEASURE_TIME_MS 20	// MINIMAL TIME TO MEASURE PULSE

#define RPM_MESSAGE_SIZE 5

#include "stm32c0xx_hal.h"
#include <stdio.h>
#include <stdbool.h>

extern TIM_HandleTypeDef htim1;

unsigned RPM_Init(void);
unsigned RPM_GetData(uint8_t aRPMMessage[5]);
unsigned RPMtoRPMMessage(uint8_t aRPMMessage[RPM_MESSAGE_SIZE], float *aRPM, uint16_t *aTime, uint8_t aID);
unsigned RPM_UpdateFrequencyRatio(float aSYNCFrequencyRatio);


#endif /* _RPM_H */
