#include "rpm.h"

static uint32_t clock=0;
unsigned MX_RPM_Init(TIM_HandleTypeDef *htim)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  htim->Instance = TIM1;
  htim->Init.Prescaler = 0;
  htim->Init.CounterMode = TIM_COUNTERMODE_UP;
  htim->Init.Period = 65535;
  htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim->Init.RepetitionCounter = 0;
  htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(htim) != HAL_OK){return 1;}

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(htim, &sMasterConfig) != HAL_OK){return 2;}

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(htim, &sConfigIC, TIM_CHANNEL_4) != HAL_OK){return 3;}

  TIM1_Start_IT(htim);
  clock=HAL_RCC_GetPCLK1Freq();
  return 0;
}



static void TIM1_Start_IT(TIM_HandleTypeDef *htim)
{
    HAL_TIM_IC_Start_IT(htim, TIM_CHANNEL_4);
	HAL_TIM_Base_Start_IT(htim);
}


static uint16_t captureValue = 0,previousCaptureValue = 0;
static float frequency = 0;
static float rpm=0;
static volatile bool flag=0;
static uint32_t ticks=0;
static unsigned int overflows=0;
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
    	__disable_irq();
        captureValue = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
        //__HAL_TIM_SET_COUNTER(&htim1, 0);
    	//tady tento pristup ma nejmensi chybu, ale nutno osetrit overflow

        if(overflows){
            ticks=65535*overflows + captureValue - previousCaptureValue;
            __NOP();
            overflows=0;
        }
        else{
        	ticks=captureValue - previousCaptureValue;
        	__NOP();
        }
        frequency = clock / ticks;
        rpm=(frequency*60)/PULSES;
        previousCaptureValue = captureValue;
        //flag=1;

        __enable_irq();
    }
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM1){
		__NOP();
		if (__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_UPDATE)) {overflows++;}

	}
}


void RPM_Process(void){

}
