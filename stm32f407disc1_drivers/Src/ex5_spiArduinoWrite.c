#include "stm32f407disc1.h"
#include "stm32f407disc1_gpio_driver.h"
#include "stm32f407disc1_spi_driver.h"

#include "string.h"

void ButtonInit(void);
void SpiPinInit(void);
void SpiInterfaceInit(void);

void delay(void){
	for(int i = 0; i < 10000; i++);
}

int main(void){

	ButtonInit();
	SpiPinInit();
	SpiInterfaceInit();
	while(1){}
	return 0;
}


void ButtonInit(void){

	GPIOx_Handle_t GPIO_LED;
	GPIOx_Handle_t GPIO_BUTTON;

	GPIO_LED.pGPIOx = DISC_GPIOD;
	GPIO_BUTTON.pGPIOx = DISC_GPIOA;

	GPIO_LED.GPIOx_PinConfig.GPIOx_PinNo = GPIOx_PINNO_12;
	GPIO_BUTTON.GPIOx_PinConfig.GPIOx_PinNo = GPIOx_PINNO_0;

	GPIO_LED.GPIOx_PinConfig.GPIOx_PinMode = GPIOx_PINMODE_OUTPUT;
	GPIO_BUTTON.GPIOx_PinConfig.GPIOx_PinMode = GPIOx_PINMODE_INT_RET;

	GPIO_LED.GPIOx_PinConfig.GPIOx_PinOutType = GPIOx_PINOUTTYPE_PP;
	GPIO_BUTTON.GPIOx_PinConfig.GPIOx_PinOutType = GPIOx_PINOUTTYPE_PP;
	GPIO_LED.GPIOx_PinConfig.GPIOx_PinResControl = GPIOx_PINRESCONTROL_NPUPD;
	GPIO_BUTTON.GPIOx_PinConfig.GPIOx_PinResControl = GPIOx_PINRESCONTROL_PD;

	GPIO_LED.GPIOx_PinConfig.GPIOx_PinSpeed = GPIOx_PINSPEED_M;
	GPIO_BUTTON.GPIOx_PinConfig.GPIOx_PinSpeed = GPIOx_PINSPEED_M;

	GPIO_Init(&GPIO_LED);
	GPIO_Init(&GPIO_BUTTON);

	GPIO_IRQConfig(DISC_EXTIx_IRQ(0), DISC_IRQPRIORITY_15, ENABLE);

}

void SpiPinInit(void){

	GPIOx_Handle_t SPI_PIN;

	SPI_PIN.pGPIOx = DISC_GPIOB;

	SPI_PIN.GPIOx_PinConfig.GPIOx_PinMode = GPIOx_PINMODE_ALT;
	SPI_PIN.GPIOx_PinConfig.GPIOx_PinAltMode = GPIOx_AF_5;
	SPI_PIN.GPIOx_PinConfig.GPIOx_PinOutType = GPIOx_PINOUTTYPE_PP;
	SPI_PIN.GPIOx_PinConfig.GPIOx_PinResControl = GPIOx_PINRESCONTROL_NPUPD;
	SPI_PIN.GPIOx_PinConfig.GPIOx_PinSpeed = GPIOx_PINSPEED_H;

	//PB15 -> MOSI
	SPI_PIN.GPIOx_PinConfig.GPIOx_PinNo = GPIOx_PINNO_15;
	GPIO_Init(&SPI_PIN);

	//PB12 -> NSS
	SPI_PIN.GPIOx_PinConfig.GPIOx_PinNo = GPIOx_PINNO_12;
	GPIO_Init(&SPI_PIN);

	//PB13 -> SCK
	SPI_PIN.GPIOx_PinConfig.GPIOx_PinNo = GPIOx_PINNO_13;
	GPIO_Init(&SPI_PIN);

}

void SpiInterfaceInit(void){

	SPIx_Handle_t SPI2;

	SPI2.pSPIx = DISC_SPI2I2S2;

	SPI2.SPIx_PinConfig.SPIx_DeviceMode = SPIx_DEVICEMODE_MASTER;
	SPI2.SPIx_PinConfig.SPIx_BusConfig = SPIx_BUSCONFG_FULLDUPLEX;
	SPI2.SPIx_PinConfig.SPIx_DFF = SPIx_DFF_8BIT;
	SPI2.SPIx_PinConfig.SPIx_CPHA = SPIx_CPHA_LOW;
	SPI2.SPIx_PinConfig.SPIx_CPOL = SPIx_CPOL_LEAD;
	SPI2.SPIx_PinConfig.SPIx_SCLKSpeed = SPIx_SCLKSPEED_DIV8;
	SPI2.SPIx_PinConfig.SPIx_SSM = SPIx_SSM_HSM;

	SPI_Init(&SPI2);

}

void EXTI0_IRQHandler(void){

	GPIO_IRQHandle(GPIOx_PINNO_0);
	GPIO_TogglePin(DISC_GPIOD, GPIOx_PINNO_12);
	delay();

	SPI_Status(DISC_SPI2I2S2, ENABLE);

	char Data[] = "Hello World";
	uint8_t DataLen = strlen(Data);

	SPI_SendData(DISC_SPI2I2S2, &DataLen, 1);

	SPI_SendData(DISC_SPI2I2S2, (uint8_t*) Data, (uint32_t)DataLen);

	//lets confirm SPI is not busy
	while( SPIGetFlagStatus(DISC_SPI2I2S2,SPIx_FLAG_BSY) );

	//Disable the SPI2 peripheral
	SPI_Status(DISC_SPI2I2S2,DISABLE);
}




