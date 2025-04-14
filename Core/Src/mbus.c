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

uint16_t timerTop=320;
uint16_t timerFrameDuration=10;


// INTERNAL: FRAME CHECKSUM SUBFUNCTION
void static HALCPU_CRC_CRC16CCITT(const uint8_t aData[], const uint16_t aSize, uint8_t aCRCresult[2],uint16_t aCRCinput) {
    for (uint16_t i = 0; i < aSize; i++) {
    	aCRCinput = HALCPU_CRC_CRC16CCITT_LUT8B_au16[(uint8_t)(aData[i] ^ (aCRCinput >> 8))] ^ (aCRCinput << 8);
    }
    aCRCresult[0] = (uint8_t)(aCRCinput & 0xFF);
    aCRCresult[1] = (uint8_t)((aCRCinput >> 8) & 0xFF);
}

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

	HALCPU_CRC_CRC16CCITT(tmpFrameBuffer,CurrentFrameSize,aFCS,0xFFFF);
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

// INTERNAL: RESET COMMUNICATION
void static MBUS_Reset(void){
	HAL_TIM_OC_Stop_IT(&htim3, TIM_CHANNEL_1);
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
	if (HAL_UARTEx_DisableFifoMode(&huart) != HAL_OK) {return 4;}
	MBUS_FrameBuffer=calloc(1,sizeof(struct FrameBuffer));
	MBUS_FrameBuffer_ToTransmit=calloc(1,sizeof(struct FrameBuffer));
	MBUS_FrameBuffer_Processed=calloc(1,sizeof(struct FrameBuffer));
	if (MBUS_FrameBuffer== NULL || MBUS_FrameBuffer_ToTransmit== NULL || MBUS_FrameBuffer_Processed== NULL) {return 5;}

	// MBUS TIMEOUT TIMER INIT
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 2500;
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
	sConfigOC.Pulse = timerTop;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK){return 10;}

	// START COMMUNICATION
	MBUS_Reset();
	return 0;
}

void MBUS_StartTimeoutTimer(void)
{
	HAL_TIM_OC_Stop_IT(&htim3, TIM_CHANNEL_1);		// STOP TIMEOUT TIMER
	if(MBUS_FrameStatus==Payload){__HAL_TIM_SET_COUNTER(&htim3,timerTop-MBUS_FrameBuffer->PS[0]*timerFrameDuration);}
	else if(MBUS_FrameStatus==FCS){__HAL_TIM_SET_COUNTER(&htim3,timerTop-2*timerFrameDuration);}
	else{__HAL_TIM_SET_COUNTER(&htim3,timerTop-timerFrameDuration);}
	HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);		// START TIMEOUT TIMER

}

// INTERNAL: INTERRUPT - UART FINISHED TRANSMITTING
 void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1){
    	__disable_irq();
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
    	    	}
    	    	else{
    	    		HAL_UART_Transmit_IT(huart,MBUS_FrameBuffer->CHID, sizeof(MBUS_FrameBuffer->CHID));
    	    		MBUS_FrameStatus=CHID;
    	    	}
        		break;

    	    case CHID:
    	        // MASTER:
        		HAL_UART_Transmit_IT(huart,MBUS_FrameBuffer->PS, sizeof(MBUS_FrameBuffer->PS));
	        	MBUS_FrameStatus=PS;
        		break;

    	    case PS:
    	    	// MASTER:
    	    	HAL_UART_Transmit_IT(huart,MBUS_FrameBuffer->Payload,MBUS_FrameBuffer->PS[0]);
	        	MBUS_FrameStatus=Payload;
    	        break;

    	    case Payload:
    	    	// MASTER:
    	    	HAL_UART_Transmit_IT(huart,MBUS_FrameBuffer->FCS, sizeof(MBUS_FrameBuffer->FCS));
	        	MBUS_FrameStatus=FCS;
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
    	__enable_irq();
    }
}

// INTERNAL: INTERRUPT - UART FINISHED RECEIVING
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1){
    	__disable_irq();
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
		        else if (MBUS_FrameBuffer->TXID[0]>=0x00 && MBUS_FrameBuffer->TXID[0]<=0x0E){	// check validity of TXID
		        	HAL_UART_Receive_IT(huart,MBUS_FrameBuffer->RXID, sizeof(MBUS_FrameBuffer->RXID));
			        MBUS_FrameStatus=RXID;
		        }
		        // RESET
		        else{	// invalid TXID or SYNC byte
		        	if(MBUS_FrameBuffer->TXID[0]>=SYNCbyte){MBUS_UpdateProcessedBuffer(SYNC_BYTE);}
		        	else{MBUS_UpdateProcessedBuffer(RX_ERROR);}
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
	        	HAL_UART_Receive_IT(huart,MBUS_FrameBuffer->Payload,MBUS_FrameBuffer->PS[0]);
	        	MBUS_FrameStatus=Payload;
		        break;

		    case Payload:
		    	// SLAVE:
	        	HAL_UART_Receive_IT(huart,MBUS_FrameBuffer->FCS,sizeof(MBUS_FrameBuffer->FCS));
	    		CalculateFCS(tmpFCSbuffer,MBUS_FrameBuffer);
	        	MBUS_FrameStatus=FCS;
	    		break;

		    case FCS:
		    	// SLAVE:
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
    	__enable_irq();
	}
}

// INTERNAL: BREAK EVENT
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	if (HAL_UART_ERROR_FE) {
    	__disable_irq();
    	HAL_UART_Receive_IT(huart,MBUS_FrameBuffer->TXID, sizeof(MBUS_FrameBuffer->TXID));
    	MBUS_FrameStatus=TXID;
    	__enable_irq();
    }
}

// TIMER TIMEOUT
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM3){
    	__disable_irq();
		__HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);
		if(MBUS_FrameStatus==ACK){MBUS_UpdateProcessedBuffer(TX_NO_ACK);}
		else{MBUS_UpdateProcessedBuffer(ERROR_TIMEOUT);}
		MBUS_Reset();
    	__enable_irq();
	}
}

