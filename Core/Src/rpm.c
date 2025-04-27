#include "rpm.h"

TIM_HandleTypeDef htim1;
static volatile bool bNewRPM = false;
static volatile bool bMaxMeasureTimeExceeded = false;

static uint32_t CLK_default;
static uint32_t CLK;
volatile static uint8_t RPM_ID = 0;
static uint16_t OVFlimit;
volatile static uint16_t captureValue = 0,previousCaptureValue = 0, measureTime=0;
volatile static uint32_t ticks=0;
volatile static float RPM=0;
volatile static uint8_t minMeasureTime,maxMeasureTime;
volatile static uint32_t OVFcounter=0;
static volatile uint8_t measureCounter=0;



unsigned RPM_Init(void){
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK){return 1;}

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK){return 2;}

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_4) != HAL_OK){return 3;}

  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4);
  HAL_TIM_Base_Start_IT(&htim1);

  CLK_default=HAL_RCC_GetPCLK1Freq();
  CLK=CLK_default;
  OVFlimit=(CLK*60)/(RPM_THRESHOLD*65536*RPM_PULSES);
  minMeasureTime=((uint32_t)((float)(OVFlimit/3)*65536))/(CLK/1000);
  maxMeasureTime=((uint32_t)(float)OVFlimit*65536)/(CLK/1000);
  return 0;
}

void RPM_ResetMeasurement(void)
{
	previousCaptureValue = captureValue;
	OVFcounter=0;
	measureCounter=0;
	bNewRPM=true;
}

unsigned RPM_GetData(uint8_t aRPMMessage[5])
{
	if(!bNewRPM){return 1;}					// no new data
	if(aRPMMessage == NULL){return 2;}		// invalid pointer
	RPMtoRPMMessage(aRPMMessage,&RPM,&measureTime,RPM_ID);
	RPM_ID++;
	bNewRPM=false;
	return 0;
}

unsigned RPM_UpdateFrequencyRatio(float aSYNCFrequencyRatio)
{
	if(!aSYNCFrequencyRatio){return 1;}
	CLK = (float)CLK_default * (aSYNCFrequencyRatio / 100.0f);
	OVFlimit=(CLK*60)/(RPM_THRESHOLD*65536*RPM_PULSES);
	minMeasureTime=((uint32_t)((float)(OVFlimit/3)*65536))/(CLK/1000);
	maxMeasureTime=((uint32_t)(float)OVFlimit*65536)/(CLK/1000);

	return 0;
}

unsigned RPMtoRPMMessage(uint8_t aRPMMessage[RPM_MESSAGE_SIZE], float *aRPM, uint16_t *aTime, uint8_t aID)
{
	if(*aRPM>255){return 1;}	// invalid data
	if(*aTime>65535){return 2;}	// invalid data

	/**
		DATA FORMAT:
		n = XX.YY rpm
		[0] RPM_h - XX
		[1] RPM_l - YY
		measure time = T ms
		[2] time_h - GG = T/256
		[3] time_l - HH = T%256
		[4] settings: 0-optical, 1-Hall
	**/
	if(aID>255){return 3;}	// invalid data
	if(aRPMMessage == NULL){return 4;}
	aRPMMessage[0]=(uint8_t)*aRPM;
	aRPMMessage[1]=(uint8_t)(float)((*aRPM-aRPMMessage[0])*100);
	aRPMMessage[2]=(uint8_t)(*aTime >> 8);		// H Byte
	aRPMMessage[3]=(uint8_t)(*aTime & 0xFF);	// L Byte
	aRPMMessage[4]=aID;
	return 0;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) //
    {
		//__disable_irq();
		captureValue = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
		if(OVFcounter>0){ticks=65536*OVFcounter + captureValue - previousCaptureValue;}
		else{ticks=captureValue - previousCaptureValue;}
		measureTime=((float)ticks/(float)(CLK/1000));	// measureTime in ms
		measureCounter++;
		if(bMaxMeasureTimeExceeded)
		{
			RPM=0;
			RPM_ResetMeasurement();
			bMaxMeasureTimeExceeded=false;
		}
		else if(measureTime>minMeasureTime)
		{
			RPM=((float)(CLK/1000)*60*measureCounter)/((float)(ticks/1000)*RPM_PULSES);
			if(RPM > RPM_LIMIT){	// invalid value
				RPM=255.255;
			}
			RPM_ResetMeasurement();
		}
		//__enable_irq();
    }
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM1){
		//__disable_irq();
		if (__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_UPDATE))
		{
			if(OVFcounter>=OVFlimit)
			{
				captureValue = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
				measureTime=maxMeasureTime;
				RPM=0;	// value below threshold
				RPM_ResetMeasurement();
				bMaxMeasureTimeExceeded=true;
			}
			else{OVFcounter++;}
		}
		//__enable_irq();
	}
}







