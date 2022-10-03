/*
 * ex3_interrupt.c
 *
 *  Created on: Jul 7, 2022
 *      Author: USER
 */

#include <stm32f407disc1.h>
#include <stm32f407disc1_gpio_driver.h>


int main(void){

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

	while(1){}
	return 0;

}

void EXTI0_IRQHandler(void){
	GPIO_IRQHandle(GPIOx_PINNO_0);
	GPIO_TogglePin(DISC_GPIOD, GPIOx_PINNO_12);
}






