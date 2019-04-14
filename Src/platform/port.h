#ifndef _PORT_H_
#define _PORT_H_

#include <stdio.h>
#include <string.h>
#include "main.h"
#include "stm32f0xx.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_def.h"
#include "stm32f0xx_ll_exti.h"
#include "deca_device_api.h"
#include "deca_types.h"
#include "deca_spi.h"
#include "Deca_device_api.h"
#include "deca_regs.h"

#define BUFFLEN 	(512)

void reset_DW1000(void);
void spi_set_rate_low(void);
void spi_set_rate_high(void);

uint32_t port_GetEXT_IRQStatus(void);
uint32_t port_CheckEXT_IRQ(void);
void port_DisableEXT_IRQ(void);
void port_EnableEXT_IRQ(void);
void port_SPI_Write(uint8_t* pData, uint16_t Size);
void port_SPI_Read(uint8_t* pData, uint16_t Size);
#endif

