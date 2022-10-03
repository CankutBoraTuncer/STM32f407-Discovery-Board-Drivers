

#include "stm32f407disc1_gpio_driver.h"


/*******************************************************************
 * @function					- GPIO_PCLKControl
 *
 * @brief						- Enables or disables the peripheral clock of the GPIO port
 *
 * @param[in]					- Base address of GPIO
 * @param[in]					- ENABLE or DISABLE macros
 *
 *
 * @return						- none
 *
 * @Note						- none
 *
 */
void GPIO_PCLKControl(GPIOx_Configuration_t *pGPIOx, uint8_t status){
	if(status == ENABLE){
		if(pGPIOx==DISC_GPIOA){
			DISC_GPIOA_PCLK_EN();
		}
		else if(pGPIOx==DISC_GPIOB){
			DISC_GPIOB_PCLK_EN();
		}
		else if(pGPIOx==DISC_GPIOC){
			DISC_GPIOC_PCLK_EN();
		}
		else if(pGPIOx==DISC_GPIOD){
			DISC_GPIOD_PCLK_EN();
		}
		else if(pGPIOx==DISC_GPIOE){
			DISC_GPIOE_PCLK_EN();
		}
		else if(pGPIOx==DISC_GPIOF){
			DISC_GPIOF_PCLK_EN();
		}
		else if(pGPIOx==DISC_GPIOG){
			DISC_GPIOG_PCLK_EN();
		}
		else if(pGPIOx==DISC_GPIOH){
			DISC_GPIOH_PCLK_EN();
		}
		else if(pGPIOx==DISC_GPIOI){
			DISC_GPIOI_PCLK_EN();
		}
		else if(pGPIOx==DISC_GPIOJ){
			DISC_GPIOJ_PCLK_EN();
		}
		else if(pGPIOx==DISC_GPIOK){
			DISC_GPIOK_PCLK_EN();
		}
	}
	else{
		if(pGPIOx==DISC_GPIOA){
			DISC_GPIOA_PCLK_DIS();
		}
		else if(pGPIOx==DISC_GPIOB){
			DISC_GPIOB_PCLK_DIS();
		}
		else if(pGPIOx==DISC_GPIOC){
			DISC_GPIOC_PCLK_DIS();
		}
		else if(pGPIOx==DISC_GPIOD){
			DISC_GPIOD_PCLK_DIS();
		}
		else if(pGPIOx==DISC_GPIOE){
			DISC_GPIOE_PCLK_DIS();
		}
		else if(pGPIOx==DISC_GPIOF){
			DISC_GPIOF_PCLK_DIS();
		}
		else if(pGPIOx==DISC_GPIOG){
			DISC_GPIOG_PCLK_DIS();
		}
		else if(pGPIOx==DISC_GPIOH){
			DISC_GPIOH_PCLK_DIS();
		}
		else if(pGPIOx==DISC_GPIOI){
			DISC_GPIOI_PCLK_DIS();
		}
		else if(pGPIOx==DISC_GPIOJ){
			DISC_GPIOJ_PCLK_DIS();
		}
		else if(pGPIOx==DISC_GPIOK){
			DISC_GPIOK_PCLK_DIS();
		}
	}
}









/*******************************************************************
 * @function					- GPIO_Init
 *
 * @brief						- Initializes the GPIO port
 *
 * @param[in]					- Base address of GPIO
 *
 *
 *
 * @return						- none
 *
 * @Note						- none
 *
 */
void GPIO_Init(GPIOx_Handle_t *pGPIOx_Handle){

	uint32_t temp = 0;

	GPIO_PCLKControl(pGPIOx_Handle->pGPIOx, ENABLE);

	// configure gpio pin
	if(pGPIOx_Handle->GPIOx_PinConfig.GPIOx_PinMode <= GPIOx_PINMODE_ANALOG){
		//non interrupt
		temp = (pGPIOx_Handle->GPIOx_PinConfig.GPIOx_PinMode << (2 * pGPIOx_Handle->GPIOx_PinConfig.GPIOx_PinNo));
		pGPIOx_Handle->pGPIOx->GPIO_MODER |= temp;
		temp = 0;

	}
	else{
		if(pGPIOx_Handle->GPIOx_PinConfig.GPIOx_PinMode == GPIOx_PINMODE_INT_FET){
			DISC_EXTI->EXTI_FTSR |= (1 << pGPIOx_Handle->GPIOx_PinConfig.GPIOx_PinNo);
			DISC_EXTI->EXTI_RTSR &= ~(1 << pGPIOx_Handle->GPIOx_PinConfig.GPIOx_PinNo);
		}
		else if(pGPIOx_Handle->GPIOx_PinConfig.GPIOx_PinMode == GPIOx_PINMODE_INT_RET){
			DISC_EXTI->EXTI_FTSR &= ~(1 << pGPIOx_Handle->GPIOx_PinConfig.GPIOx_PinNo);
			DISC_EXTI->EXTI_RTSR |= (1 << pGPIOx_Handle->GPIOx_PinConfig.GPIOx_PinNo);
		}
		else if(pGPIOx_Handle->GPIOx_PinConfig.GPIOx_PinMode == GPIOx_PINMODE_INT_REFET){
			DISC_EXTI->EXTI_FTSR |= (1 << pGPIOx_Handle->GPIOx_PinConfig.GPIOx_PinNo);
			DISC_EXTI->EXTI_RTSR |= (1 << pGPIOx_Handle->GPIOx_PinConfig.GPIOx_PinNo);
		}

			uint8_t temp1 = pGPIOx_Handle->GPIOx_PinConfig.GPIOx_PinNo / 4;
			uint8_t temp2 = pGPIOx_Handle->GPIOx_PinConfig.GPIOx_PinNo % 4;
			uint16_t temp3 = GPIO_TO_NUMBER(pGPIOx_Handle->pGPIOx);

			DISC_SYSCFG_PCLK_EN();

			DISC_SYSCFG->SYSCFG_EXTICR[temp1] |= temp3 << (temp2 * 4);
			DISC_EXTI->EXTI_IMR |= 1 << pGPIOx_Handle -> GPIOx_PinConfig.GPIOx_PinNo;

		}

	// configure speed
	temp = (pGPIOx_Handle->GPIOx_PinConfig.GPIOx_PinSpeed << (2 * pGPIOx_Handle->GPIOx_PinConfig.GPIOx_PinNo));
	pGPIOx_Handle->pGPIOx->GPIO_OSPEEDR &= ~(0x3 << (2 * pGPIOx_Handle->GPIOx_PinConfig.GPIOx_PinNo));
	pGPIOx_Handle->pGPIOx->GPIO_OSPEEDR |= temp;
	temp = 0;

	// configure pull up pull down
	temp = (pGPIOx_Handle->GPIOx_PinConfig.GPIOx_PinResControl << (2 * pGPIOx_Handle->GPIOx_PinConfig.GPIOx_PinNo));
	pGPIOx_Handle->pGPIOx->GPIO_PUPDR &= ~(0x3 << (2 * pGPIOx_Handle->GPIOx_PinConfig.GPIOx_PinNo));
	pGPIOx_Handle->pGPIOx->GPIO_PUPDR |= temp;
	temp = 0;

	// configure output type
	temp = (pGPIOx_Handle->GPIOx_PinConfig.GPIOx_PinOutType << (pGPIOx_Handle->GPIOx_PinConfig.GPIOx_PinNo));
	pGPIOx_Handle->pGPIOx->GPIO_OTYPER &= ~(0x1 << (pGPIOx_Handle->GPIOx_PinConfig.GPIOx_PinNo));
	pGPIOx_Handle->pGPIOx->GPIO_OTYPER |= temp;
	temp = 0;

	// configure alt functionality
	if(pGPIOx_Handle->GPIOx_PinConfig.GPIOx_PinMode == GPIOx_PINMODE_ALT){

		uint8_t temp3 = pGPIOx_Handle->GPIOx_PinConfig.GPIOx_PinNo / 8;
		uint8_t temp4 = pGPIOx_Handle->GPIOx_PinConfig.GPIOx_PinNo  % 8;
		pGPIOx_Handle->pGPIOx->GPIO_AFR[temp3] &= ~(0xF << ( 4 * temp4 ) ); //clearing
		pGPIOx_Handle->pGPIOx->GPIO_AFR[temp3] |= (pGPIOx_Handle->GPIOx_PinConfig.GPIOx_PinAltMode << ( 4 * temp4 ) );

		temp = 0;
	}
}









/*******************************************************************
 * @function					- GPIO_DeInit
 *
 * @brief						- DeInitializes the GPIO port
 *
 * @param[in]					- Base address of GPIO
 *
 *
 *
 * @return						- none
 *
 * @Note						- none
 *
 */
void GPIO_DeInit(GPIOx_Configuration_t *pGPIOx){
	if(pGPIOx==DISC_GPIOA){
		DISC_GPIOA_PCLK_RST();
	}
	else if(pGPIOx==DISC_GPIOB){
		DISC_GPIOB_PCLK_RST();
	}
	else if(pGPIOx==DISC_GPIOC){
		DISC_GPIOC_PCLK_RST();
	}
	else if(pGPIOx==DISC_GPIOD){
		DISC_GPIOD_PCLK_RST();
	}
	else if(pGPIOx==DISC_GPIOE){
		DISC_GPIOE_PCLK_RST();
	}
	else if(pGPIOx==DISC_GPIOF){
		DISC_GPIOF_PCLK_RST();
	}
	else if(pGPIOx==DISC_GPIOG){
		DISC_GPIOG_PCLK_RST();
	}
	else if(pGPIOx==DISC_GPIOH){
		DISC_GPIOH_PCLK_RST();
	}
	else if(pGPIOx==DISC_GPIOI){
		DISC_GPIOI_PCLK_RST();
	}
	else if(pGPIOx==DISC_GPIOJ){
		DISC_GPIOJ_PCLK_RST();
	}
	else if(pGPIOx==DISC_GPIOK){
		DISC_GPIOK_PCLK_RST();
	}
}









//Read
/*******************************************************************
 * @function					- GPIO_ReadPin
 *
 * @brief						- Reads the pin of the GPIO port
 *
 * @param[in]					- Base address of GPIO
 * @param[in]					- Pin number
 *
 *
 * @return						- The input result of the pin 0 or 1
 *
 * @Note						- none
 *
 */
uint8_t GPIO_ReadPin(GPIOx_Configuration_t *pGPIOx, uint8_t pinNumber){
	uint8_t temp = (uint8_t)(((pGPIOx->GPIO_IDR) & ((uint32_t)1<<pinNumber)) >> pinNumber);
	return temp;
}









/*******************************************************************
 * @function					- GPIO_ReadPort
 *
 * @brief						- Reads the port of the GPIO port
 *
 * @param[in]					- Base address of GPIO
 *
 *
 *
 * @return						- The input result of the port 16 bit
 *
 * @Note						- none
 *
 */
uint16_t GPIO_ReadPort(GPIOx_Configuration_t *pGPIOx){
	uint16_t temp = (uint16_t)(pGPIOx->GPIO_IDR);
	return temp;
}










/*******************************************************************
 * @function					- GPIO_WritePort
 *
 * @brief						- Writes to the port of the GPIO port
 *
 * @param[in]					- Base address of GPIO
 * @param[in]					- Data to send
 *
 *
 * @return						- none
 *
 * @Note						- none
 *
 */
void GPIO_WritePort(GPIOx_Configuration_t *pGPIOx, uint16_t data){
	pGPIOx->GPIO_ODR = data;
}









/*******************************************************************
 * @function					- GPIO_WritePin
 *
 * @brief						- Writes to the pin of the GPIO port
 *
 * @param[in]					- Base address of GPIO
 * @param[in]					- Pin number
 * @param[in]					- Data to send
 *
 * @return						- none
 *
 * @Note						- none
 *
 */
void GPIO_WritePin(GPIOx_Configuration_t *pGPIOx, uint8_t pinNumber, uint8_t data){
	if(data == GPIO_PIN_SET){
		pGPIOx->GPIO_ODR |= (1 << pinNumber);
	}
	if(data == GPIO_PIN_RST){
		pGPIOx->GPIO_ODR &= ~(1 << pinNumber);
	}
}










/*******************************************************************
 * @function					- GPIO_TogglePin
 *
 * @brief						- Toggles to the pin of the GPIO port
 *
 * @param[in]					- Base address of GPIO
 * @param[in]					- Pin number
 *
 *
 * @return						- none
 *
 * @Note						- none
 *
 */
void GPIO_TogglePin(GPIOx_Configuration_t *pGPIOx, uint8_t pinNumber){
	pGPIOx->GPIO_ODR ^= (1 << pinNumber);
}

//IRQ
/*******************************************************************
 * @function					- GPIO_IRQConfig
 *
 * @brief						- Configuration of the GPIO port
 *
 * @param[in]					- IRQ vector number
 * @param[in]					- IRQ priority, 0 the highest, 3 the lowest priority
 * @param[in]					- ENABLE or DISABLE macros
 *
 * @return						- none
 *
 * @Note						- none
 *
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t state){

	if(state == ENABLE){
		if(IRQNumber <= 31){
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(32 <= IRQNumber && IRQNumber <= 61){
			*NVIC_ISER1 |= (1 << IRQNumber % 32);
		}
		else if(62 <= IRQNumber && IRQNumber <= 95){
			*NVIC_ISER2 |= (1 << IRQNumber % 62);
		}

	}
	else{
		if(IRQNumber <= 31){
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(32 <= IRQNumber && IRQNumber <= 61){
			*NVIC_ICER1 |= (1 << IRQNumber % 32);
		}
		else if(62 <= IRQNumber && IRQNumber <= 95){
			*NVIC_ICER2 |= (1 << IRQNumber % 62);
		}
	}

	uint8_t temp = IRQNumber / 4;
	uint8_t temp1 = IRQNumber % 4;

	*(NVIC_PR_BASE_ADDR + temp) |= (IRQPriority << (8 * temp1 + 4));

}
/*******************************************************************
 * @function					- GPIO_IRQHandle
 *
 * @brief						- ISR of the GPIO Interrupt
 *
 * @param[in]					- Pin number
 *
 *
 *
 * @return						- none
 *
 * @Note						- none
 *
 */
void GPIO_IRQHandle(uint8_t pinNumber){

	if(DISC_EXTI->EXTI_PR & (1 << pinNumber)){
		DISC_EXTI->EXTI_PR |= (1 << pinNumber);
	}

}
