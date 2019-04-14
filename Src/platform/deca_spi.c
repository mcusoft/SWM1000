/*! ----------------------------------------------------------------------------
 * @file	deca_spi.c
 * @brief	SPI access functions
 *
 * @attention
 *
 * Copyright 2015 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */

#include "deca_spi.h"
#include "deca_device_api.h"
#include "port.h"


/*! ------------------------------------------------------------------------------------------------------------------
 * Function: openspi()
 *
 * Low level abstract function to open and initialise access to the SPI device.
 * returns 0 for success, or -1 for error
 */
int openspi(/*SPI_TypeDef* SPIx*/)
{
	// done by port.c, default SPI used is SPI1

	return 0;

} // end openspi()

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: closespi()
 *
 * Low level abstract function to close the the SPI device.
 * returns 0 for success, or -1 for error
 */
int closespi(void)
{

	return 0;

} // end closespi()

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: writetospi()
 *
 * Low level abstract function to write to the SPI
 * Takes two separate byte buffers for write header and write data
 * returns 0 for success, or -1 for error
 */
int writetospi
(
	uint16		 headerLength,
	const uint8 *headerBuffer,
	uint32		 bodyLength,
	const uint8 *bodyBuffer
)
{
    decaIrqStatus_t  stat ;
    stat = decamutexon() ;

	HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_RESET); /**< Put chip select line low */

	port_SPI_Write((uint8_t *)headerBuffer, headerLength);
	port_SPI_Write((uint8_t *)bodyBuffer, bodyLength);

	HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_SET); /**< Put chip select line high */

    decamutexoff(stat) ;

    return 0;
} // end writetospi()


/*! ------------------------------------------------------------------------------------------------------------------
 * Function: readfromspi()
 *
 * Low level abstract function to read from the SPI
 * Takes two separate byte buffers for write header and read data
 * returns the offset into read buffer where first byte of read data may be found,
 * or returns -1 if there was an error
 */
int readfromspi
(
	uint16		headerLength,
	const uint8 *headerBuffer,
	uint32		readlength,
	uint8	   *readBuffer
)
{
	//uint8_t spi_TmpBuffer[BUFFLEN];
	assert_param(headerLength+readlength < BUFFLEN );
	
    decaIrqStatus_t  stat ;
    stat = decamutexon() ;

	HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_RESET); /**< Put chip select line low */

	port_SPI_Write((uint8_t*)headerBuffer, headerLength);
	port_SPI_Read((uint8_t*)readBuffer, readlength);

	HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_SET); /**< Put chip select line high */

	decamutexoff(stat);

    return 0;
} // end readfromspi()

