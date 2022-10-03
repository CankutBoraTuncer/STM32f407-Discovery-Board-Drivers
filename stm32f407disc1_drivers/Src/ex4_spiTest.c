/*
 * ex4_spiTest.c
 *
 *  Created on: Jul 19, 2022
 *      Author: USER
 */


#include <stm32f407disc1.h>
#include <stm32f407disc1_spi_driver.h>
#include <stm32f407disc1_gpio_driver.h>

#include <string.h>

void SPI_GPIO_Init(void){

	GPIOx_Handle_t SPIPin;

	SPIPin.pGPIOx = DISC_GPIOB;

	SPIPin.GPIOx_PinConfig.GPIOx_PinAltMode = GPIOx_AF_5;
	SPIPin.GPIOx_PinConfig.GPIOx_PinMode = GPIOx_PINMODE_ALT;
	SPIPin.GPIOx_PinConfig.GPIOx_PinOutType = GPIOx_PINOUTTYPE_PP;
	SPIPin.GPIOx_PinConfig.GPIOx_PinResControl = GPIOx_PINRESCONTROL_NPUPD;
	SPIPin.GPIOx_PinConfig.GPIOx_PinSpeed = GPIOx_PINSPEED_M;

	//SCLK
	SPIPin.GPIOx_PinConfig.GPIOx_PinNo = GPIOx_PINNO_13;
	GPIO_Init(&SPIPin);

	//MOSI
	SPIPin.GPIOx_PinConfig.GPIOx_PinNo = GPIOx_PINNO_15;
	GPIO_Init(&SPIPin);

	//MISO
	SPIPin.GPIOx_PinConfig.GPIOx_PinNo = GPIOx_PINNO_14;
	GPIO_Init(&SPIPin);

	//NSS
	SPIPin.GPIOx_PinConfig.GPIOx_PinNo = GPIOx_PINNO_12;
	GPIO_Init(&SPIPin);
}

void SPI2_Init(void){

	SPIx_Handle_t SPI2;

	SPI2.pSPIx = DISC_SPI2I2S2;

	SPI2.SPIx_PinConfig.SPIx_BusConfig = SPIx_BUSCONFG_FULLDUPLEX;
	SPI2.SPIx_PinConfig.SPIx_CPHA = SPIx_CPHA_LOW;
	SPI2.SPIx_PinConfig.SPIx_CPOL = SPIx_CPOL_LEAD;
	SPI2.SPIx_PinConfig.SPIx_DFF = 	SPIx_DFF_8BIT;
	SPI2.SPIx_PinConfig.SPIx_DeviceMode = SPIx_DEVICEMODE_MASTER;
	SPI2.SPIx_PinConfig.SPIx_SCLKSpeed = SPIx_SCLKSPEED_DIV2;
	SPI2.SPIx_PinConfig.SPIx_SSM = SPIx_SSM_HSM;

	SPI_Init(&SPI2);

}

int main(void){

	const char TestString[] = "Hello World";

	//Configure Pins
	SPI_GPIO_Init();

	//Configure spi
	SPI2_Init();

	//Enable spi
	SPI_Status(DISC_SPI2I2S2, ENABLE);

	//Send Data spi
	SPI_SendData(DISC_SPI2I2S2, (uint8_t *)TestString, strlen(TestString));

	//Disable spi
	SPI_Status(DISC_SPI2I2S2, DISABLE);

	while(1);

	return 0;
}





























