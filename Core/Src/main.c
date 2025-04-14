/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

void SystemClock_Config(void);
static void MX_GPIO_Init(void);

unsigned counterTX_NO_ACK=0;
unsigned counterTX_IDLE_FRAME=0;
unsigned counterELSE=0;

uint32_t last_time=0;
uint32_t elapsed=0;

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();

  if (RPM_Init()){Error_Handler();}
  if (MBUS_Init()){Error_Handler();}

  struct FrameBuffer *ReceivedFrameBuffer=calloc(1,sizeof(struct FrameBuffer));
  if(ReceivedFrameBuffer==NULL){Error_Handler();}

  uint8_t *RPMMessage=calloc(RPM_MESSAGE_SIZE,sizeof(uint8_t));
  if(RPMMessage==NULL){Error_Handler();}

  last_time=HAL_GetTick();
  while (1)
  {
	  if(!RPM_GetData(RPMMessage))
	  {
		  elapsed=HAL_GetTick()-last_time;
		  last_time=HAL_GetTick();
		  if(!MBUS_SetTransmittedData(MBUSBridgeID,CHIDrpm,RPM_MESSAGE_SIZE,RPMMessage,true))
		  {
			  __NOP();
		  }
	  }


	  if(MBUS_GetProcessedFrame(ReceivedFrameBuffer)==TX_NO_ACK){counterTX_NO_ACK++;}
	  else if(MBUS_GetProcessedFrame(ReceivedFrameBuffer)==TX_IDLE_FRAME){counterTX_IDLE_FRAME++;}
	  else(counterELSE++);

	  // HAL_Delay(1000);

  }
}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_0);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
}

void Error_Handler(void)
{
 __disable_irq();
  while (1)
  {
	  __NOP();
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
