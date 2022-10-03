#include <stdio.h>
#include "stm32f407disc1.h"
#include "stm32f407disc1_gpio_driver.h"
#include "stm32f407disc1_spi_driver.h"

void SpiPinConf(void);
void SpiInterfaceConf(void);

void ButtonPinConf(void);
void LEDPinConf(void);
uint8_t ParseData(SPIx_Handle_t *pSPIx_Handle);

#define BUTTON_PIN_NO 0
#define LED_PIN_NO 12

#define COMMAND_SEND_START 0x50
#define COMMAND_SEND_END   0x51

#define COMMAND_RCV_ACK    0
#define COMMAND_RCV_NACK   1
#define COMMAND_RCV_GOOD   2
#define COMMAND_RCV_ERR    3

SPIx_Handle_t SPI2;
uint8_t recieved;
uint8_t startCommand = 0x50;

int main(void){

	SpiPinConf();
	ButtonPinConf();
	LEDPinConf();

	SpiInterfaceConf();

	while(1){

	}

	return 0;

}

void SpiInterfaceConf(void){

	SPI2.pSPIx = DISC_SPI2I2S2;

	SPI2.SPIx_PinConfig.SPIx_BusConfig = SPIx_BUSCONFG_FULLDUPLEX;
	SPI2.SPIx_PinConfig.SPIx_CPHA = SPIx_CPHA_LOW;
	SPI2.SPIx_PinConfig.SPIx_CPOL = SPIx_CPOL_LEAD;
	SPI2.SPIx_PinConfig.SPIx_DFF = SPIx_DFF_8BIT;
	SPI2.SPIx_PinConfig.SPIx_DeviceMode = SPIx_DEVICEMODE_MASTER;
	SPI2.SPIx_PinConfig.SPIx_SCLKSpeed = SPIx_SCLKSPEED_DIV2; //8MHz
	SPI2.SPIx_PinConfig.SPIx_SSM = SPIx_SSM_HSM;

	SPI_Init(&SPI2);

	SPI_IRQConfig(DISC_SPIx_IRQ(2), DISC_IRQPRIORITY_15, ENABLE);
	SPI_Status(DISC_SPI2I2S2, ENABLE);

}

void EXTI0_IRQHandler(void){

	GPIO_IRQHandle(BUTTON_PIN_NO);
	GPIO_TogglePin(DISC_GPIOD, LED_PIN_NO);

	SPI_SendDataIT(&SPI2, &startCommand, 1);
	SPI_RecieveDataIT(&SPI2, &recieved, 1);

}

void SPI2_IRQHandler(void){
	SPI_IRQHandle(&SPI2);
}

void SPI_ApplicationCallback(SPIx_Handle_t *pSPIx_Handle, uint8_t event){

	if(event == SPI_EVENT_RX_CMPLT){
		recieved = ParseData(pSPIx_Handle);
		printf("Rcvd data = ");
		SPI_RecieveDataIT(&SPI2, &recieved, 1);
	}
	else if(event == SPI_EVENT_TX_CMPLT){

	}
	else if(event == SPI_EVENT_OVR_ERR){

	}
}

uint8_t ParseData(SPIx_Handle_t *pSPIx_Handle){

	uint8_t RxBuffer = *pSPIx_Handle->pRxBuffer;

	switch(RxBuffer){
	case 0x60:
		RxBuffer = COMMAND_RCV_ACK;
		break;
	case 0x61:
		RxBuffer = COMMAND_RCV_NACK;
		break;
	case 0x62:
		RxBuffer = COMMAND_RCV_GOOD;
		break;
	default:
		RxBuffer = COMMAND_RCV_ERR;
		break;
	}
	return RxBuffer;
}




void SpiPinConf(void){

	GPIOx_Handle_t SPI_PIN;

	SPI_PIN.pGPIOx = DISC_GPIOB;

	SPI_PIN.GPIOx_PinConfig.GPIOx_PinMode = GPIOx_PINMODE_ALT;
	SPI_PIN.GPIOx_PinConfig.GPIOx_PinAltMode = GPIOx_AF_5;
	SPI_PIN.GPIOx_PinConfig.GPIOx_PinOutType = GPIOx_PINOUTTYPE_PP;
	SPI_PIN.GPIOx_PinConfig.GPIOx_PinResControl = GPIOx_PINRESCONTROL_PU;
	SPI_PIN.GPIOx_PinConfig.GPIOx_PinSpeed = GPIOx_PINSPEED_M;

	//MISO
	SPI_PIN.GPIOx_PinConfig.GPIOx_PinNo = GPIOx_PINNO_15;
	GPIO_Init(&SPI_PIN);

	//MOSI
	SPI_PIN.GPIOx_PinConfig.GPIOx_PinNo = GPIOx_PINNO_14;
	GPIO_Init(&SPI_PIN);

	//NSS
	SPI_PIN.GPIOx_PinConfig.GPIOx_PinNo = GPIOx_PINNO_12;
	GPIO_Init(&SPI_PIN);

	//SCLK
	SPI_PIN.GPIOx_PinConfig.GPIOx_PinNo = GPIOx_PINNO_13;
	GPIO_Init(&SPI_PIN);

}

void ButtonPinConf(void){

	GPIOx_Handle_t BUTTON_PIN;

	BUTTON_PIN.pGPIOx = DISC_GPIOA;

	BUTTON_PIN.GPIOx_PinConfig.GPIOx_PinMode = GPIOx_PINMODE_INT_RET;
	BUTTON_PIN.GPIOx_PinConfig.GPIOx_PinOutType = GPIOx_PINOUTTYPE_PP;
	BUTTON_PIN.GPIOx_PinConfig.GPIOx_PinResControl = GPIOx_PINRESCONTROL_PD;
	BUTTON_PIN.GPIOx_PinConfig.GPIOx_PinSpeed = GPIOx_PINSPEED_M;


	BUTTON_PIN.GPIOx_PinConfig.GPIOx_PinNo = BUTTON_PIN_NO;
	GPIO_Init(&BUTTON_PIN);

	GPIO_IRQConfig(DISC_EXTIx_IRQ(0), DISC_IRQPRIORITY_10, ENABLE);

}

void LEDPinConf(void){

	GPIOx_Handle_t LED_PIN;

	LED_PIN.pGPIOx = DISC_GPIOD;

	LED_PIN.GPIOx_PinConfig.GPIOx_PinMode = GPIOx_PINMODE_OUTPUT;
	LED_PIN.GPIOx_PinConfig.GPIOx_PinOutType = GPIOx_PINOUTTYPE_PP;
	LED_PIN.GPIOx_PinConfig.GPIOx_PinResControl = GPIOx_PINRESCONTROL_PD;
	LED_PIN.GPIOx_PinConfig.GPIOx_PinSpeed = GPIOx_PINSPEED_M;


	LED_PIN.GPIOx_PinConfig.GPIOx_PinNo = LED_PIN_NO;
	GPIO_Init(&LED_PIN);

}




