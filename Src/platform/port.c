#include "port.h"

extern SPI_HandleTypeDef hspi1;

__IO ITStatus SpiReady = RESET;

void reset_DW1000(void)
{
	GPIO_InitTypeDef	GPIO_InitStruct;

	// Enable GPIO used for DW1000 reset as open collector output
	GPIO_InitStruct.Pin = DW_RESET_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(DW_RESET_GPIO_Port, &GPIO_InitStruct);

	//drive the RSTn pin low
	HAL_GPIO_WritePin(DW_RESET_GPIO_Port, DW_RESET_Pin, GPIO_PIN_RESET);

	//put the pin back to tri-state ... as input
	GPIO_InitStruct.Pin = DW_RESET_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(DW_RESET_GPIO_Port, &GPIO_InitStruct); 

	HAL_Delay(2);
}

void spi_set_rate_low(void)
{
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
	HAL_SPI_Init(&hspi1);
}

void spi_set_rate_high(void)
{
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
	HAL_SPI_Init(&hspi1);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

}

__INLINE void process_deca_irq(void)
{
	while(port_CheckEXT_IRQ() != 0)
	{
    	dwt_isr();

    } //while DW1000 IRQ line active
}

__INLINE void port_DisableEXT_IRQ(void)
{
	LL_EXTI_DisableEvent_0_31(DW_IRQn_Pin);
}

__INLINE void port_EnableEXT_IRQ(void)
{
	LL_EXTI_EnableEvent_0_31(DW_IRQn_Pin);
}

__INLINE uint32_t port_GetEXT_IRQStatus(void)
{
	return LL_EXTI_IsEnabledIT_0_31(DW_IRQn_Pin);
}

__INLINE uint32_t port_CheckEXT_IRQ(void)
{
	return HAL_GPIO_ReadPin(DW_IRQn_GPIO_Port, DW_IRQn_Pin);
}

void port_SPI_Write(uint8_t* pData, uint16_t Size)
{
	SpiReady = RESET;

	if(HAL_SPI_Transmit_IT(&hspi1, pData, Size) != HAL_OK)
	{
		Error_Handler();
	}

	while(SpiReady == RESET)
		;
}

void port_SPI_Read(uint8_t* pData, uint16_t Size)
{
	SpiReady = RESET;

	if(HAL_SPI_Receive_IT(&hspi1, pData, Size) != HAL_OK)
	{
		Error_Handler();
	}

	while(SpiReady == RESET)
		;
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{ 
   /* Set variable value to SET : transmission is finished*/
   SpiReady = SET;
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
  
  /* Set variable value to SET : transmission is finished*/
  SpiReady = SET;
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{ 
   /* Set variable value to SET : transmission is finished*/
   SpiReady = RESET;
 
   /* call error handler */
   Error_Handler();
}

