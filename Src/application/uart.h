#ifndef __UART_H
#define __UART_H

#include <string.h>
#include <math.h>
#include "stm32f0xx_hal.h"
#include "Deca_device_api.h"

void Uart_Init(void);
void Uart_OK(void);
void Uart_ERROR(void);
void Uart_Print_Tag(uint8_t* msg_data, uint16_t msg_lenth);
void Uart_Print_RSSI(dwt_rxdiag_t* rssi_data);
void Uart_Print_Dist(double count_data, uint8_t anchor_addr, uint8_t tag_addr);
void Uart_Send(uint8_t *pData, uint16_t Size);
void Uart_Recieve(void);
void Uart_Handle(void);

#endif

