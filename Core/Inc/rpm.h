#ifndef _RPM_H
#define _RPM_H

#define PULSES 6

#include "stm32c0xx_hal.h"
#include <stdio.h>
#include <stdbool.h>

unsigned MX_RPM_Init(TIM_HandleTypeDef *htim);
static void TIM1_Start_IT(TIM_HandleTypeDef *htim);
void RPM_Process(void);


#endif /* _RPM_H */
