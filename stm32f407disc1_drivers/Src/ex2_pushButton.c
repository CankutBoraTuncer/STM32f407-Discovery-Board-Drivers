#include <stm32f407disc1.h>
#include <stm32f407disc1_gpio_driver.h>


void delay(int time){
	for(int i = 0; i<time; i++ );
}

int main(void){

	GPIOx_Handle_t GPIO_LED;
	GPIOx_Handle_t GPIO_BUTTON;

	GPIO_LED.pGPIOx = DISC_GPIOD;
	GPIO_BUTTON.pGPIOx = DISC_GPIOA;

	GPIO_LED.GPIOx_PinConfig.GPIOx_PinMode = GPIOx_PINMODE_OUTPUT;
	GPIO_BUTTON.GPIOx_PinConfig.GPIOx_PinMode = GPIOx_PINMODE_INPUT;

	GPIO_LED.GPIOx_PinConfig.GPIOx_PinNo = GPIOx_PINNO_12;
	GPIO_BUTTON.GPIOx_PinConfig.GPIOx_PinNo = GPIOx_PINNO_0;

	GPIO_LED.GPIOx_PinConfig.GPIOx_PinSpeed = GPIOx_PINSPEED_M;
	GPIO_BUTTON.GPIOx_PinConfig.GPIOx_PinSpeed = GPIOx_PINSPEED_M;

	GPIO_LED.GPIOx_PinConfig.GPIOx_PinOutType = GPIOx_PINOUTTYPE_PP;
	GPIO_BUTTON.GPIOx_PinConfig.GPIOx_PinOutType = GPIOx_PINOUTTYPE_PP;
	GPIO_LED.GPIOx_PinConfig.GPIOx_PinResControl = GPIOx_PINRESCONTROL_NPUPD;
	GPIO_BUTTON.GPIOx_PinConfig.GPIOx_PinResControl = GPIOx_PINRESCONTROL_NPUPD;


	GPIO_Init(&GPIO_LED);
	GPIO_Init(&GPIO_BUTTON);

	while(1){
		while(GPIO_ReadPin(GPIO_BUTTON.pGPIOx, GPIO_BUTTON.GPIOx_PinConfig.GPIOx_PinNo)){
			GPIO_WritePin(GPIO_LED.pGPIOx, GPIO_LED.GPIOx_PinConfig.GPIOx_PinNo, SET);
			delay(50000);
		}
		GPIO_WritePin(GPIO_LED.pGPIOx, GPIO_LED.GPIOx_PinConfig.GPIOx_PinNo, RST);
		/*
		if(GPIO_ReadPin(GPIO_BUTTON.pGPIOx, GPIO_BUTTON.GPIOx_PinConfig.GPIOx_PinNo )){
			GPIO_TogglePin(GPIO_LED.pGPIOx, GPIO_LED.GPIOx_PinConfig.GPIOx_PinNo, SET);
			delay(50000);
		}*/
	}



}

