#include "mbus.h"

TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart;

enum FrameStructure MBUS_FrameStatus;
enum ProcessedFrameStatus MBUS_ProcessedFrameStatus;

struct FrameBuffer *MBUS_FrameBuffer;
struct FrameBuffer *MBUS_FrameBuffer_ToTransmit;
struct FrameBuffer *MBUS_FrameBuffer_Processed;

const uint8_t ACKbyte=0xA5;
uint8_t tmpFCSbuffer[2];
volatile static bool bNewDataToTransmit=false;
volatile static bool bNewProcessedData=false;
volatile static uint8_t Payload_UARTCount=0;
volatile static uint8_t FCS_UARTCount=0;

static uint16_t SYNCbytePeriod = 0;
static float SYNCFrequencyRatio=0;

volatile static uint16_t UART_EdgeTime,UART_EdgePeriodStartTime,UART_EdgePeriodStopTime;
volatile static uint8_t UART_EdgeCounter=0;
volatile static uint32_t UART_Error;


// INTERNAL: FRAME CHECKSUM CALCULATION
void static CalculateFCS(uint8_t aFCS[2],struct FrameBuffer *aFrameBuffer){
	uint16_t CurrentFrameSize=4*sizeof(uint8_t)+MBUS_FrameBuffer->PS[0];
	uint8_t *tmpFrameBuffer;
	tmpFrameBuffer=malloc(CurrentFrameSize*sizeof(uint8_t));
	tmpFrameBuffer[0]=aFrameBuffer->TXID[0];
	tmpFrameBuffer[1]=aFrameBuffer->RXID[0];
	tmpFrameBuffer[2]=aFrameBuffer->CHID[0];
	tmpFrameBuffer[3]=aFrameBuffer->PS[0];
	for (uint16_t i = 0 ; i < aFrameBuffer->PS[0]; i++ ){
		tmpFrameBuffer[4+i]=aFrameBuffer->Payload[i];	// copy data from Payload to tmpFrameBuffer
	}

	//HALCPU_CRC_CRC16CCITT(tmpFrameBuffer,CurrentFrameSize,aFCS,0xFFFF);

	uint16_t tmpCRC=0xFFFF;
    for (uint16_t i = 0; i < CurrentFrameSize; i++) {
    	tmpCRC = HALCPU_CRC_CRC16CCITT_LUT8B_au16[(uint8_t)(tmpFrameBuffer[i] ^ (tmpCRC >> 8))] ^ (tmpCRC << 8);
    }
    aFCS[0] = (uint8_t)(tmpCRC & 0xFF);
    aFCS[1] = (uint8_t)((tmpCRC >> 8) & 0xFF);

	free(tmpFrameBuffer);
}

// API: UPDATE DATA TO TRANSMIT
unsigned MBUS_SetTransmittedData(const uint8_t aRXID,const uint8_t aCHID,const uint8_t aPayloadSize,const uint8_t aPayload[PAYLOAD_MAX_SIZE],const bool aAlowOverwrite){
	if(	bNewDataToTransmit && !aAlowOverwrite){return 1;}	// last data not processed && overwrite is not allowed

	MBUS_FrameBuffer_ToTransmit->TXID[0]=myID;
	if(aRXID>=0x00 && aRXID<=0x0F && aRXID!=myID){MBUS_FrameBuffer_ToTransmit->RXID[0]=aRXID;}	// check argument validity
	else{return 2;}

	MBUS_FrameBuffer_ToTransmit->CHID[0]=aCHID;

	if(aPayloadSize>0 && aPayloadSize<=PAYLOAD_MAX_SIZE && aPayload){	// check argument validity
		MBUS_FrameBuffer_ToTransmit->PS[0]=aPayloadSize;
		memcpy(MBUS_FrameBuffer_ToTransmit->Payload,aPayload,aPayloadSize);
	}
	else{return 3;}

	CalculateFCS(MBUS_FrameBuffer_ToTransmit->FCS,MBUS_FrameBuffer_ToTransmit);
	MBUS_FrameBuffer_ToTransmit->ACK[0]=0xFF;
	bNewDataToTransmit=true;
	return 0;
}

// API: FOR MAIN
enum ProcessedFrameStatus MBUS_GetProcessedFrame(struct FrameBuffer *aFrameBuffer){
	if(bNewProcessedData){
		if(aFrameBuffer== NULL){return ERROR_INVALID_POINTER;}	// invalid pointer
		aFrameBuffer=MBUS_FrameBuffer_Processed;
		bNewProcessedData=false;
		return MBUS_ProcessedFrameStatus;
	}
	else{return NO_NEW_DATA;}
}

// API: RETURNS FREQUENCY CALIBRATION RATIO *100
float MBUS_GetFrequencySYNCRatio(void)
{
	//SYNCFrequencyRatio=(100.0f*SYNCbyteDefaultPeriod)/SYNCbytePeriod;//(float)
	SYNCFrequencyRatio=(100.0f*SYNCbytePeriod)/SYNCbyteDefaultPeriod;//(float)
	if(SYNCFrequencyRatio<=SYNCFrequencyRatioMax && SYNCFrequencyRatio>=SYNCFrequencyRatioMin){return SYNCFrequencyRatio;}
	else{return 0.00;}
}

// INTERNAL: INPUT DATA TO TRANSMIT
void static MBUS_UpdateTransmittedBuffer(void){
	if(bNewDataToTransmit){	// no new data -> transmit IDLE frame
		MBUS_FrameBuffer=MBUS_FrameBuffer_ToTransmit;
		bNewDataToTransmit=false;
	}
	else{MBUS_FrameBuffer->RXID[0]=IDLEframe;}
}

// INTERNAL: RECEIVED DATA TO OUTPUT BUFFER
void static MBUS_UpdateProcessedBuffer(enum ProcessedFrameStatus aProcessedFrameStatus){
	MBUS_FrameBuffer_Processed=MBUS_FrameBuffer;
	MBUS_ProcessedFrameStatus = aProcessedFrameStatus;
	bNewProcessedData=true;
}

void MBUS_StartTimeoutTimer(void)
{
	__HAL_TIM_SET_COUNTER(&htim3,0);

	//HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);		// START TIMEOUT TIMER
	HAL_TIM_Base_Start(&htim3);

	if(MBUS_FrameStatus==TXID)
	{
		HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
		UART_EdgeCounter=0;
	}
}

void MBUS_StopTimeoutTimer(void)
{

	//HAL_TIM_OC_Stop_IT(&htim3, TIM_CHANNEL_1);		// STOP TIMEOUT TIMER
	HAL_TIM_Base_Stop(&htim3);
	if(MBUS_FrameStatus==TXID && MBUS_FrameBuffer->TXID[0]==SYNCbyte)
	{
		HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);
		SYNCbytePeriod=UART_EdgePeriodStopTime-UART_EdgePeriodStartTime;
	}
}


// INTERNAL: RESET COMMUNICATION
void static MBUS_Reset(void){
	//HAL_TIM_OC_Stop_IT(&htim3, TIM_CHANNEL_1);
	MBUS_StopTimeoutTimer();

	HAL_UART_AbortTransmit_IT(&huart);	// not tested yet
	HAL_UART_AbortReceive_IT(&huart);	// not tested yet

	Payload_UARTCount=0;
	FCS_UARTCount=0;

	MBUS_FrameBuffer->TXID[0]=0xFF;
	HAL_UART_Receive_IT(&huart,MBUS_FrameBuffer->BREAK, sizeof(MBUS_FrameBuffer->BREAK));
	MBUS_FrameStatus=IDLE;
}

// API: INIT FUNCTION FOR UART AND TIMEOUT TIMER
unsigned MBUS_Init(void) {
	// MBUS UART INIT
	huart.Instance = USART1;
	huart.Init.BaudRate = 57600;
	huart.Init.WordLength = UART_WORDLENGTH_8B;
	huart.Init.StopBits = UART_STOPBITS_1;
	huart.Init.Parity = UART_PARITY_NONE;
	huart.Init.Mode = UART_MODE_TX_RX;
	huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart.Init.OverSampling = UART_OVERSAMPLING_16;
	huart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart) != HAL_OK) {return 1;}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) {return 2;}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) {return 3;}
	//if (HAL_UARTEx_DisableFifoMode(&huart) != HAL_OK) {return 4;}

	if (HAL_LIN_Init(&huart,UART_LINBREAKDETECTLENGTH_11B)) {return 11;}	// BREAK DETECT LENGTH
	//HAL_UART_ReceiverTimeout_Config(&huart,5);

	MBUS_FrameBuffer=calloc(1,sizeof(struct FrameBuffer));
	MBUS_FrameBuffer_ToTransmit=calloc(1,sizeof(struct FrameBuffer));
	MBUS_FrameBuffer_Processed=calloc(1,sizeof(struct FrameBuffer));
	if (MBUS_FrameBuffer== NULL || MBUS_FrameBuffer_ToTransmit== NULL || MBUS_FrameBuffer_Processed== NULL) {return 5;}

	// MBUS TIMEOUT TIMER INIT
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;	// 0 pro 48 MHz
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	//htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK){return 6;}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK){return 7;}
	if (HAL_TIM_OC_Init(&htim3) != HAL_OK){return 8;}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK){return 9;}
	sConfigOC.OCMode = TIM_OCMODE_TIMING;
	sConfigOC.Pulse = MBUS_MAX_FRAME_TIME;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK){return 10;}
	HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);		// INTERRUPT ON TIMER 3

	// EXTERNAL INTERRUPT FOR FRAME TIMING
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = UARTtiming_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(UARTtiming_GPIO_Port, &GPIO_InitStruct);
	HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
	//HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

	// START COMMUNICATION
	MBUS_Reset();
	return 0;
}

// INTERNAL: INTERRUPT - UART FINISHED TRANSMITTING
 void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1){
    	//__disable_irq();
    	MBUS_StopTimeoutTimer();
    	switch (MBUS_FrameStatus) {
    	    case IDLE:
    	    	break;

    	    case TXID:
    	    	// FAULT:
    	    	MBUS_Reset();
    	    	break;

    	    case RXID:
    	    	// MASTER:
    	    	if(MBUS_FrameBuffer->RXID[0]==IDLEframe){
	    			MBUS_UpdateProcessedBuffer(TX_IDLE_FRAME);
    	    		MBUS_Reset();	// transmitted IDLE frame -> wait for new communication
    	    		break;
    	    	}
    	    	else{
    	    		HAL_UART_Transmit_IT(huart,MBUS_FrameBuffer->CHID, sizeof(MBUS_FrameBuffer->CHID));
    	    		MBUS_FrameStatus=CHID;
    	    		break;
    	    	}

    	    case CHID:
    	        // MASTER:
        		HAL_UART_Transmit_IT(huart,MBUS_FrameBuffer->PS, sizeof(MBUS_FrameBuffer->PS));
	        	MBUS_FrameStatus=PS;
        		break;

    	    case PS:
    	    	// MASTER:
    	    	HAL_UART_Transmit_IT(huart,MBUS_FrameBuffer->Payload+Payload_UARTCount,sizeof(uint8_t));
	    		Payload_UARTCount++;
    	    	if(Payload_UARTCount>=MBUS_FrameBuffer->PS[0])
    	    	{
        	    	Payload_UARTCount=0;
        	    	MBUS_FrameStatus=Payload;
    	    	}
    	        break;

    	    case Payload:
    	    	// MASTER:
    	    	HAL_UART_Transmit_IT(huart,MBUS_FrameBuffer->FCS+FCS_UARTCount, sizeof(uint8_t));
	    		FCS_UARTCount++;
    	    	if(FCS_UARTCount>=2)
    	    	{
        	    	FCS_UARTCount=0;
        	    	MBUS_FrameStatus=FCS;
    	    	}
    	    	break;

    	    case FCS:
    	    	// MASTER:
    	    	if(myID==MBUS_FrameBuffer->TXID[0] && MBUS_FrameBuffer->RXID[0]!=RXIDbroadcast){	// after frame transmission I expect ACK byte
    	        	HAL_UART_Receive_IT(huart,MBUS_FrameBuffer->ACK, sizeof(MBUS_FrameBuffer->ACK));
    	        	MBUS_FrameStatus=ACK;
    	    	}
    	    	else{	// fault or RXID is broadcast
	    			MBUS_UpdateProcessedBuffer(TX_OK);
    	    		MBUS_Reset();
    	    	}
    	        break;

    	    case ACK:
    	    	// SLAVE:
    			MBUS_UpdateProcessedBuffer(RX_OK);
    	    	MBUS_Reset();
    	        break;

    	    default:
    	    	MBUS_Reset();
    	    	break;
    	}
    	MBUS_StartTimeoutTimer();
    	//__enable_irq();
    }
}




// INTERNAL: INTERRUPT - UART FINISHED RECEIVING
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1){
    	//__disable_irq();
    	MBUS_StopTimeoutTimer();
		switch (MBUS_FrameStatus) {
		    case IDLE:
				break;

		    case TXID:
		    	// MASTER:
		        if(MBUS_FrameBuffer->TXID[0]==myID){
		        	MBUS_UpdateTransmittedBuffer();
	        		HAL_UART_Transmit_IT(huart,MBUS_FrameBuffer->RXID, sizeof(MBUS_FrameBuffer->RXID));
		        	MBUS_FrameStatus=RXID;
		        }
		        // SLAVE:
	        	else if(MBUS_FrameBuffer->TXID[0]==SYNCbyte){	// SYNC Byte
		        		MBUS_UpdateProcessedBuffer(SYNC_BYTE);
			        	MBUS_Reset();
	        	}
		        else if (MBUS_FrameBuffer->TXID[0]>=0x00 && MBUS_FrameBuffer->TXID[0]<=0x0E){	// check validity of TXID
		        	HAL_UART_Receive_IT(huart,MBUS_FrameBuffer->RXID, sizeof(MBUS_FrameBuffer->RXID));
		        	MBUS_FrameStatus=RXID;
		        }
		        else{	// invalid TXID
		        	MBUS_UpdateProcessedBuffer(RX_ERROR);
		        	MBUS_Reset();
		        }
		        break;

		    case RXID:
		    	// SLAVE:
		        if(myID==MBUS_FrameBuffer->RXID[0] || RXIDbroadcast ==MBUS_FrameBuffer->RXID[0]){	// received RXID is equal to myID or broadcast
		        	HAL_UART_Receive_IT(huart,MBUS_FrameBuffer->CHID, sizeof(MBUS_FrameBuffer->CHID));
		        	MBUS_FrameStatus=CHID;
		        }
		        else{	// frame is not for me
		        	MBUS_UpdateProcessedBuffer(RX_NOT_MY_ADDRESS);
		        	MBUS_Reset();
		        }
		        break;

		    case CHID:
		    	// SLAVE:
	        	HAL_UART_Receive_IT(huart,MBUS_FrameBuffer->PS, sizeof(MBUS_FrameBuffer->PS));
	        	MBUS_FrameStatus=PS;
		        break;

		    case PS:
		    	// SLAVE:
	        	HAL_UART_Receive_IT(huart,MBUS_FrameBuffer->Payload+Payload_UARTCount,sizeof(uint8_t));
	    		Payload_UARTCount++;
    	    	if(Payload_UARTCount>=MBUS_FrameBuffer->PS[0])
    	    	{
        	    	Payload_UARTCount=0;
    	    		MBUS_FrameStatus=Payload;
    	    	}
		        break;

		    case Payload:
		    	// SLAVE:
    	    	HAL_UART_Receive_IT(huart,MBUS_FrameBuffer->FCS+FCS_UARTCount, sizeof(uint8_t));
	    		FCS_UARTCount++;
    	    	if(FCS_UARTCount>=2)
    	    	{
        	    	FCS_UARTCount=0;
        	    	MBUS_FrameStatus=FCS;
    	    	}
	    		break;

		    case FCS:
		    	// SLAVE:
	        	CalculateFCS(tmpFCSbuffer,MBUS_FrameBuffer);
		    	if(MBUS_FrameBuffer->FCS[0]==tmpFCSbuffer[0] && MBUS_FrameBuffer->FCS[1]==tmpFCSbuffer[1]){	// compare FCS
		    		if(MBUS_FrameBuffer->RXID[0]==RXIDbroadcast){	// if RXID is BROADCAST -> do not ACK
		    			MBUS_UpdateProcessedBuffer(RX_OK);
		    			MBUS_Reset();
		    	        break;
		    		}
		    		else{
		    			HAL_UART_Transmit_IT(huart, &ACKbyte, sizeof(ACKbyte));	// transmit ACK
		    		}
		    	}
		    	else{	// invalid FCS
	    			MBUS_UpdateProcessedBuffer(RX_ERROR);
		    		MBUS_Reset();
		    		break;
		    	}
	        	MBUS_FrameStatus=ACK;
		        break;

		    case ACK:
		    	// MASTER:
		    	if(myID==MBUS_FrameBuffer->TXID[0]){
		    		if(MBUS_FrameBuffer->ACK[0]==ACKbyte){
		    			MBUS_UpdateProcessedBuffer(TX_OK);
		    		}
		    		else{
		    			MBUS_UpdateProcessedBuffer(TX_NO_ACK);	// communication error: no ACK
		    		}
		    	}
		    	MBUS_Reset();
    	    	break;

		    default:
		    	MBUS_Reset();
		    	break;
		    }
    	MBUS_StartTimeoutTimer();
    	//__enable_irq();
	}
}


// INTERNAL: BREAK EVENT
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	UART_Error = HAL_UART_GetError(huart);

	if (UART_Error==HAL_UART_ERROR_FE)
	{
    	//__disable_irq();
    	HAL_UART_Receive_IT(huart,MBUS_FrameBuffer->TXID, sizeof(MBUS_FrameBuffer->TXID));
    	MBUS_FrameStatus=TXID;
		MBUS_StartTimeoutTimer();
    	//__enable_irq();

	}
	/*
	else if(UART_Error==HAL_UART_ERROR_RTO)
	{
    	__disable_irq();
		if(MBUS_FrameStatus==ACK){
			MBUS_UpdateProcessedBuffer(TX_NO_ACK);
		}
		else{MBUS_UpdateProcessedBuffer(ERROR_TIMEOUT);}
		MBUS_Reset();
    	__enable_irq();
	}*/
	else
	{
		MBUS_Reset();
	}
}


// TIMER TIMEOUT
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM3){
		__HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);
		if(MBUS_FrameStatus==ACK){
			MBUS_UpdateProcessedBuffer(TX_NO_ACK);
		}
		else{MBUS_UpdateProcessedBuffer(ERROR_TIMEOUT);}
		MBUS_Reset();
	}
}

// UART TIMING
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
	UART_EdgeTime=__HAL_TIM_GET_COUNTER(&htim3);

	if(UART_EdgeCounter==1){
		UART_EdgePeriodStartTime=UART_EdgeTime;
	}
	else if(UART_EdgeCounter==2){	// 1 perioda signalu
		UART_EdgePeriodStopTime=UART_EdgeTime;
	}

	UART_EdgeCounter++;
}



