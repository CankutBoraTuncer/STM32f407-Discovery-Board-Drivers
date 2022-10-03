#include "stm32f407disc1.h"
#include "stm32f407disc1_gpio_driver.h"
#include "stm32f407disc1_spi_driver.h"

#include "string.h"

#define COMMAND_LED_CNTR 					0x50
#define COMMAND_SENSOR_READ					0x51
#define RESPONSE_ACK						0x55
#define RESPONSE_NACK						0x56

#define LED_PIN								5
#define LED_ON								1
#define LED_OFF								0

void ButtonInit(void);
void SpiPinInit(void);
void SpiInterfaceInit(void);
uint8_t SpiParseResponse(uint8_t response);

void delay(void){
	for(int i = 0; i < 100000; i++);
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

	//PB14 -> SCK
	SPI_PIN.GPIOx_PinConfig.GPIOx_PinNo = GPIOx_PINNO_14;
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

	static const uint8_t DUMMY_WRITE = 0xFF;
	static uint8_t DUMMY_READ;

	GPIO_IRQHandle(GPIOx_PINNO_0);
	GPIO_TogglePin(DISC_GPIOD, GPIOx_PINNO_12);
	delay();

	SPI_Status(DISC_SPI2I2S2, ENABLE);

	uint8_t Command = COMMAND_LED_CNTR;
	uint8_t DeviceResponse;

	//Writing the command
	SPI_SendData(DISC_SPI2I2S2, &Command, 1);
	SPI_RecieveData(DISC_SPI2I2S2, &DUMMY_READ, 1);

	//Reading the response
	SPI_SendData(DISC_SPI2I2S2, &DUMMY_WRITE, 1);
	SPI_RecieveData(DISC_SPI2I2S2, &DeviceResponse, 1);

	//Write the arguments
	if(SpiParseResponse(DeviceResponse)){

		uint8_t args[2] = {LED_PIN, LED_ON};
		SPI_SendData(DISC_SPI2I2S2, args, 2);
	}

	while( SPIGetFlagStatus(DISC_SPI2I2S2,SPIx_FLAG_BSY) );
	SPI_Status(DISC_SPI2I2S2,DISABLE);
}


uint8_t SpiParseResponse(uint8_t response){
	if(response == RESPONSE_ACK){
		return 1;
	}
	return 0;
}

