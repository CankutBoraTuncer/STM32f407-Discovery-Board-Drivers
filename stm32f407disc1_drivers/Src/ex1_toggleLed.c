#include "stm32f407disc1.h"
#include "stm32f407disc1_gpio_driver.h"

void delay(void){

	for(uint32_t i = 0; i<= 500000;i++){

	}
}

int main(void){

	GPIOx_Handle_t GPIO_LED;

	GPIO_LED.pGPIOx = DISC_GPIOD;
	GPIO_LED.GPIOx_PinConfig.GPIOx_PinNo = GPIOx_PINNO_15;
	GPIO_LED.GPIOx_PinConfig.GPIOx_PinSpeed = GPIOx_PINSPEED_M;
	GPIO_LED.GPIOx_PinConfig.GPIOx_PinMode = GPIOx_PINMODE_OUTPUT;
	//GPIO_LED.GPIOx_PinConfig.GPIOx_PinOutType = GPIOx_PINOUTTYPE_PP;
	GPIO_LED.GPIOx_PinConfig.GPIOx_PinOutType = GPIOx_PINOUTTYPE_OD;
	GPIO_LED.GPIOx_PinConfig.GPIOx_PinResControl = GPIOx_PINRESCONTROL_PU;

	GPIO_Init(&GPIO_LED);

	while(1){

		GPIO_TogglePin(DISC_GPIOD, GPIOx_PINNO_15);
		delay();
	}

	return 0;
}
