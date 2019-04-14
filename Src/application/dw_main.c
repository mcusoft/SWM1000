#include "port.h"
#include "dw_main.h"
#include "tag_blink.h"
#include "twr_init.h"
#include "twr_resp.h"
#include "cont_frame.h"
#include "uart.h"

extern IWDG_HandleTypeDef hiwdg;

const char STRING_DECA_BLINK[]	= "TAG BLINK\r\n";
const char STRING_TWR_INIT[]	= "TWR INIT\r\n";
const char STRING_TWR_RESP[]	= "TWR RESP\r\n";

//DECA_STUTAS_TypeDef deca_state = DECA_BLINK;
//DECA_STUTAS_TypeDef deca_state = DECA_TWR_INIT;
DECA_STUTAS_TypeDef deca_state = DECA_TWR_RESP;
//DECA_STUTAS_TypeDef deca_state = DECA_CONT_FRAME;

void dw_main(void)
{
	DW1000_Init();
	Uart_Recieve();

	DW1000_Config();

	while(1)
	{
		DW1000_Handle();

		if(HAL_IWDG_Refresh(&hiwdg) != HAL_OK)
		{
	       Error_Handler();
	  	}
	}
}

void DW1000_Init(void)
{
	do
	{
		reset_DW1000();
	}
	while(dwt_readdevid() != DWT_DEVICE_ID);
	DW1000_Config();
}

void DW1000_Config(void)
{
	switch(deca_state)
	{
		case DECA_BLINK:
			Uart_Send((uint8_t*)STRING_DECA_BLINK, strlen(STRING_DECA_BLINK));
			DW1000_Blink_Config();
			break;
		case DECA_TWR_INIT:
			Uart_Send((uint8_t*)STRING_TWR_INIT, strlen(STRING_TWR_INIT));
			TWR_Init_Config();
			break;
		case DECA_TWR_RESP:
			Uart_Send((uint8_t*)STRING_TWR_RESP, strlen(STRING_TWR_RESP));
			TWR_Resp_Config();
			break;
		case DECA_CONT_FRAME:
			DW1000_Cont_Frame_Config();
			break;
		default:
			break;
	}
}

void DW1000_Handle(void)
{
	switch(deca_state)
	{
		case DECA_BLINK:
			DW1000_Blink_Handle();
			break;
		case DECA_TWR_INIT:
			TWR_Init_Handle();
			break;
		case DECA_TWR_RESP:
			TWR_Resp_Handle();
			break;
		case DECA_CONT_FRAME:
			DW1000_Cont_Frame_Handle();
			break;
		default:
			break;
	}
}

