#include "mbus.h"

TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart;

enum MBUS_FrameStructure MBUS_FrameStatus;

struct FrameBuffer *MBUS_FrameBuffer;
struct FrameBuffer *MBUS_FrameBuffer_ToTransmit;
struct FrameBuffer *MBUS_FrameBuffer_Received;

uint8_t FCS_calc_buffer[2];

const uint8_t myID=0x0A;	//example
const uint8_t broadcastID=0x0F;
const uint8_t ACKbyte=0xA5;
const uint8_t SYNCbyte=0x55;
const uint8_t CHIDdebug=0x07;
const uint8_t CHIDmonitor=0x06;


/**
 * The following channel IDs are reserved for standard Miele layer III protocols:
• 0x01: Update (COM_UPDATE)
• 0x02: Motor control
• 0x03: SSACP (COM_SSACP)
• 0x04: System Check
• 0x05: DSM
• 0x06: Monitor (COM_MON)
• 0x07: Debug Output
• 0x08: Object protocol (COM_OBJ)
• 0x09: Data Object Protocol 2 (DOP2)
 **/

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
	//if (!tmpFrameBuffer) {return 1;}

	tmpFrameBuffer[0]=aFrameBuffer->TXID[0];
	tmpFrameBuffer[1]=aFrameBuffer->RXID[0];
	tmpFrameBuffer[2]=aFrameBuffer->CHID[0];
	tmpFrameBuffer[3]=aFrameBuffer->PS[0];
	for (uint16_t i = 0 ; i < aFrameBuffer->PS[0]; i++ ){
		tmpFrameBuffer[4+i]=aFrameBuffer->Payload[i];	// copy data from Payload to Frame_buffer
	}

	HALCPU_CRC_CRC16CCITT(tmpFrameBuffer,CurrentFrameSize,aFCS,0xFFFF);
	free(tmpFrameBuffer);
}

// API: UPDATE DATA TO TRANSMIT
unsigned MBUS_UpdateTransmittedData(const uint8_t aRXID,const uint8_t aCHID,const uint8_t aPayloadSize,const uint8_t aPayload[PAYLOAD_MAX_SIZE]){
	MBUS_FrameBuffer_ToTransmit->TXID[0]=myID;
	if(aRXID>=0x00 && aRXID<=0x0F && aRXID!=myID){	// invalid arguments
		MBUS_FrameBuffer_ToTransmit->RXID[0]=aRXID;
	}
	else{return 1;}

	MBUS_FrameBuffer_ToTransmit->CHID[0]=aCHID;

	if(aPayloadSize>0 && aPayloadSize<=PAYLOAD_MAX_SIZE && aPayload){	// invalid arguments
		MBUS_FrameBuffer_ToTransmit->PS[0]=aPayloadSize;
		memcpy(MBUS_FrameBuffer_ToTransmit->Payload,aPayload,aPayloadSize);
	}
	else{return 2;}

	CalculateFCS(MBUS_FrameBuffer_ToTransmit->FCS,MBUS_FrameBuffer_ToTransmit);
	return 0;
}

// API: FOR MAIN
unsigned MBUS_GetProcessedData(struct FrameBuffer *aFrameBuffer){	// ale pozor abych si tam potom nesahal z mainu na ta data, kdyz dostanu tu adresu
	// OVERENi VALIDITY DAT a PRIPADNE JESTE SEPAROVANI
	// informace o tom, jestli ty data jsou novy

	// radsi asi predat primo kopii dat nez pointer, aby se mi tam na to nesahalo

	if(aFrameBuffer==MBUS_FrameBuffer_Received){	// SAME DATA
		return 1;
	}
	else{
		// HANDLING NA TO, JESTLI JSEM HO ODVYSILAL NEBO HO PRIJAL
		aFrameBuffer=MBUS_FrameBuffer_Received;
		return 0;
	}
}

// INTERNAL: INPUT DATA TO TRANSMIT
void static MBUS_FillTransmitBuffer(void){
	MBUS_FrameBuffer=MBUS_FrameBuffer_ToTransmit;
}

// INTERNAL: RECEIVED DATA TO OUTPUT
void static MBUS_FillReceivedBuffer(void){
	MBUS_FrameBuffer_Received=MBUS_FrameBuffer;
}

// INTERNAL: RESET COMMUNICATION
void static MBUS_Reset(void){
	//HAL_TIM_Base_Stop_IT(&htim3);

	//HAL_TIM_OC_Stop_IT(&htim3, TIM_CHANNEL_1);
    //__HAL_TIM_SET_COUNTER(&htim3, 0);	// RESET TIMEOUT TIMER

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
	MBUS_FrameBuffer_Received=calloc(1,sizeof(struct FrameBuffer));
	if (!MBUS_FrameBuffer || !MBUS_FrameBuffer_ToTransmit || !MBUS_FrameBuffer_Received) {return 5;}


	// MBUS TIMEOUT INIT
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 4;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_OC_Init(&htim3) != HAL_OK){return 6;}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK){return 7;}
	sConfigOC.OCMode = TIM_OCMODE_TIMING;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK){return 8;}
	//HAL_TIM_Base_Start_IT(&htim3);
	//HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);

	// START COMMUNICATIOn
	MBUS_Reset();
	return 0;
}

// INTERNAL: BREAK EVENT
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	if (HAL_UART_ERROR_FE) {	//huart->ErrorCode &
		__HAL_UART_CLEAR_FEFLAG(huart);
    	//HAL_TIM_Base_Start_IT(&htim3);

		HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);

    	HAL_UART_Receive_IT(huart,MBUS_FrameBuffer->TXID, sizeof(MBUS_FrameBuffer->TXID));
    	MBUS_FrameStatus=TXID;
    }
}

// INTERNAL: INTERRUPT - UART FINISHED TRANSMITTING
 void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1){
    	switch (MBUS_FrameStatus) {
    	    case IDLE:
    	    	MBUS_Reset();
    	        break;

    	    case TXID:
    	    	// FAULT:
    	    	MBUS_Reset();
    	    	break;

    	    case RXID:
    	    	// MASTER:
        		HAL_UART_Transmit_IT(huart,MBUS_FrameBuffer->CHID, sizeof(MBUS_FrameBuffer->CHID));
	        	MBUS_FrameStatus=CHID;
        		break;

    	    case CHID:
    	        // MASTER:
        		HAL_UART_Transmit_IT(huart,MBUS_FrameBuffer->PS, sizeof(MBUS_FrameBuffer->PS));
	        	MBUS_FrameStatus=PS;
        		break;

    	    case PS:
    	    	// MASTER:
    	    	HAL_UART_Transmit_IT(huart,MBUS_FrameBuffer->Payload,MBUS_FrameBuffer->PS[0]);	// odesilam pocet Bytu, co jsem nastavil v PS
	        	MBUS_FrameStatus=Payload;
    	        break;

    	    case Payload:
    	    	// MASTER:
    	    	HAL_UART_Transmit_IT(huart,MBUS_FrameBuffer->FCS, sizeof(MBUS_FrameBuffer->FCS));	// odvysilani Checksum po celem Frame
	        	MBUS_FrameStatus=FCS;
        		break;

    	    case FCS:
    	    	// MASTER:
    	    	if(myID==MBUS_FrameBuffer->TXID[0] && MBUS_FrameBuffer->RXID[0]!=broadcastID){	// po odvysilani celeho Frame ocekavam ACK, pokud to neni broadcast
    	        	HAL_UART_Receive_IT(huart,MBUS_FrameBuffer->ACK, sizeof(MBUS_FrameBuffer->ACK));
    	        	MBUS_FrameStatus=ACK;
    	    	}
    	    	else{	// FAULT OR RXID IS BROADCAST
    	    		MBUS_Reset();
    	    	}
    	        break;

    	    case ACK:
    	    	// SLAVE:
    	    	MBUS_Reset();
    	        break;

    	    default:
    	    	MBUS_Reset();
    	    	break;
    	}

    }
}

// INTERNAL: INTERRUPT - UART FINISHED RECEIVING
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1){
		switch (MBUS_FrameStatus) {
		    case IDLE:
		    	//MBUS_Reset();
				break;

		    case TXID:
		    	// MASTER:
		        if(MBUS_FrameBuffer->TXID[0]==myID){
		        	MBUS_FillTransmitBuffer();
	        		HAL_UART_Transmit_IT(huart,MBUS_FrameBuffer->RXID, sizeof(MBUS_FrameBuffer->RXID));
		        	MBUS_FrameStatus=RXID;
		        }
		        // SLAVE:
		        else if (MBUS_FrameBuffer->TXID[0]>=0x00 && MBUS_FrameBuffer->TXID[0]<=0x0E){	//nevysilam, ale dal posloucham, jestli nejake zpravy nebudou pro me
		        	HAL_UART_Receive_IT(huart,MBUS_FrameBuffer->RXID, sizeof(MBUS_FrameBuffer->RXID));
			        MBUS_FrameStatus=RXID;
		        }
		        // RESET:
		        else{	// SYNCbyte nebo nevalidni ID -> jdu cekat na novy zacatek
		        	MBUS_Reset();
		        }
		        break;

		    case RXID:
		    	// SLAVE:
		        if(myID==MBUS_FrameBuffer->RXID[0] || broadcastID ==MBUS_FrameBuffer->RXID[0]){	// ID je moje nebo broadcast, budu prijimat Frame
		        	HAL_UART_Receive_IT(huart,MBUS_FrameBuffer->CHID, sizeof(MBUS_FrameBuffer->CHID));
		        	MBUS_FrameStatus=CHID;
		        }
		        else{	// zpravy nejsou pro me, jdu cekat na novy zacatek
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
	    		CalculateFCS(FCS_calc_buffer,MBUS_FrameBuffer);
	        	MBUS_FrameStatus=FCS;
	    		break;

		    case FCS:
		    	// MASTER:
		    	if(myID==MBUS_FrameBuffer->TXID[0]){
		    		HAL_UART_Receive_IT(huart,MBUS_FrameBuffer->ACK, sizeof(MBUS_FrameBuffer->ACK));
		    	}
		    	// SLAVE:
		    	else{
		    		if(MBUS_FrameBuffer->FCS[0]==FCS_calc_buffer[0] && MBUS_FrameBuffer->FCS[1]==FCS_calc_buffer[1]){	//srovnej checksum
		    			if(MBUS_FrameBuffer->RXID[0]==broadcastID){	// na broadcast nedavam ACK
		    				MBUS_FillReceivedBuffer();
		    				MBUS_Reset();
		    	        	break;
		    			}
		    			else{HAL_UART_Transmit_IT(huart, &ACKbyte, sizeof(ACKbyte));}	// ACk na prijate zpravy
		    		}
		    	}
	        	MBUS_FrameStatus=ACK;
		        break;

		    case ACK:
		    	// MASTER:
		    	if(myID==MBUS_FrameBuffer->TXID[0]){
		    		if(MBUS_FrameBuffer->ACK[0]==ACKbyte){
		    			MBUS_FillReceivedBuffer();	//CORRECT FRAME
		    		}
		    		else{
		    			//COM_ERR - nedostal jsem ACK na moje vysilani -> budu muset opakovat (pokud me vyzve arbitr)
		    		}
		    	}
		    	MBUS_Reset();
    	    	break;

		    default:
		    	MBUS_Reset();
		    	break;
		    }
	}
}

/*
// INTERNAL: TIMEOUT IN COMMUNICATION
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM3){
		__NOP();
		MBUS_Reset();
	}
}
*/
