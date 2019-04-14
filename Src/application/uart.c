#include "main.h"
#include "uart.h"
#include "port.h"

extern UART_HandleTypeDef huart1;

__IO ITStatus UartReady = RESET;
__IO ITStatus UartTxBusy = RESET;
extern void twr_start(void);
extern void twr_stop(void);

const char STRING_OK[]		= "$OK\r\n";
const char STRING_ERROR[]	= "$ERROR\r\n";

const char STRING_INIT[]	= "$INIT\r\n";
const char STRING_VERSION[]	= "$VER 1.02\r\n";

const char STRING_TWR_BEGIN[]		= "$TWR,Start\r\n";
const char STRING_TWR_STOP[]		= "$TWR,Stop\r\n";
const char STRING_DIST_REQUEST[]	= "$REQUEST\r\n";
const char STRING_GET_VERSION[]		= "$VERSION\r\n";

#define BUFFER_MAX_LENTH	64
char string_buff[BUFFER_MAX_LENTH];
char messege_buff[BUFFER_MAX_LENTH];
uint8_t uart_rx_data;
uint8_t rx_buff[BUFFER_MAX_LENTH];
uint8_t rx_count;
uint8_t cmd_buff[BUFFER_MAX_LENTH];
uint8_t cmd_lenth;

uint8_t uart_output_enable;

#define TAG_MESSAGE_LEHTH	27
#define DIST_MESSAGE_LEHTH	12

void Uart_Init(void)
{
	Uart_Send((uint8_t*)STRING_INIT, strlen(STRING_INIT));
}

void Uart_OK(void)
{
	Uart_Send((uint8_t*)STRING_OK, strlen(STRING_OK));
}


void Uart_ERROR(void)
{
	Uart_Send((uint8_t*)STRING_ERROR, strlen(STRING_ERROR));
}

void Uart_Print_Tag(uint8_t* msg_data, uint16_t msg_lenth)
{
	if(msg_lenth > 0x0c)
	{
		sprintf(string_buff,"$TAG,%02X%02X%02X%02X%02X%02X%02X%02X,%d,%d\r\n", msg_data[9], msg_data[8],
					msg_data[7], msg_data[6], msg_data[5], msg_data[4], msg_data[3], msg_data[2],
					((msg_data[10] & 0x01) | ((msg_data[10] & 0x02) >> 1)), (msg_data[10] & 0x04) >> 2);
		Uart_Send((uint8_t*)string_buff, TAG_MESSAGE_LEHTH);
	}
	else
	{
		sprintf(string_buff,"$TAG,%02X%02X%02X%02X%02X%02X%02X%02X,%02X\r\n", msg_data[9], msg_data[8],
			msg_data[7], msg_data[6], msg_data[5], msg_data[4], msg_data[3], msg_data[2], msg_data[1]);
		Uart_Send((uint8_t*)string_buff, TAG_MESSAGE_LEHTH-1);
	}
}

void Uart_Print_RSSI(dwt_rxdiag_t* rssi_data)
{
#if 1
	double fp_signal_power;
	double recieve_signal_power;

	fp_signal_power = 10*log10(((pow(rssi_data->firstPathAmp1,2) + pow(rssi_data->firstPathAmp2,2) 
						+ pow(rssi_data->firstPathAmp3,2))/pow(rssi_data->rxPreamCount,2))) - 121.74;
	//printf("fp_signal_power=%fdBm\n\r", fp_signal_power);
	//sprintf(string_buff,"fp_signal_power=%fdBm\n\r", fp_signal_power);
	//HAL_UART_Transmit_IT(&UartHandle, (uint8_t*)string_buff, TAG_MESSAGE_LEHTH);

	recieve_signal_power = 10*log10((rssi_data->maxGrowthCIR*pow(2,17))/pow(rssi_data->rxPreamCount,2)) - 121.74;
	//printf("recieve_signal_power=%fdBm\n\r", recieve_signal_power);
	memset(messege_buff, 0, BUFFER_MAX_LENTH);
	sprintf(messege_buff,"rssi =%.2lfdBm %.2lfdBm\r\n", fp_signal_power, recieve_signal_power);
	//HAL_UART_Transmit_IT(&UartHandle, (uint8_t*)messege_buff, strlen(string_buff));
	Uart_Send((uint8_t*)messege_buff, strlen(messege_buff));
#endif
}

void Uart_Print_Dist(double count_data, uint8_t addr_master, uint8_t addr_slave)
{
	//if(uart_output_enable)
	{
		memset(string_buff, 0, BUFFER_MAX_LENTH);
		sprintf(string_buff,"$DIST,M%d,S%d,%2.2f\r\n", addr_master, addr_slave, count_data);
		//sprintf(string_buff,"$DIST%d,%2.2f\r\n", addr_slave, count_data);
		//sprintf(string_buff,"$DIST,%2.2f\r\n", count_data);
		Uart_Send((uint8_t*)string_buff, strlen(string_buff));
		//uart_output_enable = 0;
	}
}

void Uart_Send(uint8_t *pData, uint16_t Size)
{
#if 1
	while(UartTxBusy != RESET)
	{
	}

	UartTxBusy = SET;
	if(HAL_UART_Transmit_IT(&huart1, pData, Size)!= HAL_OK)
	{
		Error_Handler();
	}
#else
	if(HAL_UART_Transmit(&huart1, pData, Size, 1000)!= HAL_OK)
	{
		Error_Handler();
	}
#endif
}

void Uart_Recieve(void)
{
	if(HAL_UART_Receive_IT(&huart1, &uart_rx_data, 1) != HAL_OK)
	{
		Error_Handler();
	}
}

void Uart_Handle(void)
{
	if(cmd_lenth > 0)
	{
		if(!memcmp(cmd_buff, STRING_TWR_BEGIN, strlen(STRING_TWR_BEGIN)))
		{
			Uart_OK();
			//twr_start();
		}
		else if(!memcmp(cmd_buff, STRING_TWR_STOP, strlen(STRING_TWR_STOP)))
		{
			Uart_OK();
			//twr_stop();
		}
		else if(!memcmp(cmd_buff, STRING_DIST_REQUEST, strlen(STRING_DIST_REQUEST)))
		{
			uart_output_enable = 1;
		}
		else if(!memcmp(cmd_buff, STRING_GET_VERSION, strlen(STRING_GET_VERSION)))
		{
			Uart_Send((uint8_t*)STRING_VERSION, strlen(STRING_VERSION));
		}
		else
		{
			Uart_ERROR();
		}

		cmd_lenth = 0;
	}
}


/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle. 
  * @note   This example shows a simple way to report end of IT Tx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: trasfer complete*/
  UartTxBusy = RESET;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	if(UartHandle->Instance == USART1)
	{
		if(uart_rx_data == '$')
			rx_count = 0;
		rx_buff[rx_count] = uart_rx_data;
		if(rx_count < BUFFER_MAX_LENTH)
			rx_count++;
		if(uart_rx_data == '\n')
		{
			memcpy(cmd_buff, rx_buff, rx_count);
			cmd_lenth = rx_count;
			rx_count = 0;
		}

		HAL_UART_Receive_IT(UartHandle, &uart_rx_data, 1);
	}
}

/**
  * @brief  UART error callbacks
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
    UartHandle->gState = HAL_UART_STATE_READY;
	UartHandle->RxState = HAL_UART_STATE_READY;
	if(HAL_UART_Receive_IT(UartHandle, &uart_rx_data, 1) != HAL_OK)
	{
	    Error_Handler();
	}
}


