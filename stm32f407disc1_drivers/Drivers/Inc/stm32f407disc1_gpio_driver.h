/*
 * stm32f407disc1_gpio_driver.h
 *
 *  Created on: Jul 6, 2022
 *      Author: USER
 */

#ifndef INC_STM32F407DISC1_GPIO_DRIVER_H_
#define INC_STM32F407DISC1_GPIO_DRIVER_H_

#include "stm32f407disc1.h"


#endif /* INC_STM32F407DISC1_GPIO_DRIVER_H_ */


typedef struct{

	uint8_t GPIOx_PinNo;
	uint8_t GPIOx_PinMode;														/*< Check possible values here: @Modes >*/
	uint8_t GPIOx_PinSpeed;														/*< Check possible values here: @Speed >*/
	uint8_t GPIOx_PinResControl;												/*< Check possible values here: @Resistor >*/
	uint8_t GPIOx_PinOutType;													/*< Check possible values here: @Output Types >*/
	uint8_t GPIOx_PinAltMode;													/*< Check possible values here: @Alternate Function >*/

}GPIOx_PinConfig_t;

typedef struct{

	GPIOx_Configuration_t *pGPIOx;												/*< Holds the base address of the GPIOx pin >*/
	GPIOx_PinConfig_t GPIOx_PinConfig;									    /*< Holds the configuration settings of the GPIOx >*/


}GPIOx_Handle_t;

/*
 * @Modes
 */

#define GPIOx_PINMODE_INPUT  		0												/*< GPIO input mode >*/
#define GPIOx_PINMODE_OUTPUT 		1												/*< GPIO output mode >*/
#define GPIOx_PINMODE_ALT    		2												/*< GPIO alternate function mode >*/
#define GPIOx_PINMODE_ANALOG 		3												/*< GPIO analog mode >*/
#define GPIOx_PINMODE_INT_FET		4												/*< GPIO falling edge triggered mode >*/
#define GPIOx_PINMODE_INT_RET		5												/*< GPIO rising edge triggered mode >*/
#define GPIOx_PINMODE_INT_REFET		6												/*< GPIO rising edge falling edge triggered mode >*/

/*
 * @Output Types
 */

#define GPIOx_PINOUTTYPE_PP	  		0												/*< GPIO pull up-down output type >*/
#define GPIOx_PINOUTTYPE_OD	  		1												/*< GPIO open drain output type >*/

/*
 * @Speed
 */
#define GPIOx_PINSPEED_L  			0												/*< GPIO low speed pin mode>*/
#define GPIOx_PINSPEED_M 			1												/*< GPIO medium speed pin mode >*/
#define GPIOx_PINSPEED_H    		2												/*< GPIO high speed pin mode >*/
#define GPIOx_PINSPEED_VH 			3												/*< GPIO very high speed pin mode >*/

/*
 * @Resistor
 */

#define GPIOx_PINRESCONTROL_NPUPD  	0												/*< GPIO No pull up pull down resistor mode>*/
#define GPIOx_PINRESCONTROL_PU 		1												/*< GPIO pull up mode >*/
#define GPIOx_PINRESCONTROL_PD    	2												/*< GPIO pull down mode >*/

/*
 * @Pins
 */

#define GPIOx_PINNO_0  				0												/*< GPIO Pin number 0>*/
#define GPIOx_PINNO_1  				1												/*< GPIO Pin number 1>*/
#define GPIOx_PINNO_2  				2												/*< GPIO Pin number 2>*/
#define GPIOx_PINNO_3  				3												/*< GPIO Pin number 3>*/
#define GPIOx_PINNO_4  				4												/*< GPIO Pin number 4>*/
#define GPIOx_PINNO_5  				5												/*< GPIO Pin number 5>*/
#define GPIOx_PINNO_6  				6												/*< GPIO Pin number 6>*/
#define GPIOx_PINNO_7  				7												/*< GPIO Pin number 7>*/
#define GPIOx_PINNO_8  				8												/*< GPIO Pin number 8>*/
#define GPIOx_PINNO_9  				9												/*< GPIO Pin number 9>*/
#define GPIOx_PINNO_10  			10												/*< GPIO Pin number 10>*/
#define GPIOx_PINNO_11  			11												/*< GPIO Pin number 11>*/
#define GPIOx_PINNO_12  			12												/*< GPIO Pin number 12>*/
#define GPIOx_PINNO_13  			13												/*< GPIO Pin number 13>*/
#define GPIOx_PINNO_14  			14												/*< GPIO Pin number 14>*/
#define GPIOx_PINNO_15  			15												/*< GPIO Pin number 15>*/

/*
 * @Alternate Function
 */

#define GPIOx_AF_0  				0												/*< GPIO Alternate function 0>*/
#define GPIOx_AF_1  				1												/*< GPIO Alternate function 1>*/
#define GPIOx_AF_2  				2												/*< GPIO Alternate function 2>*/
#define GPIOx_AF_3  				3												/*< GPIO Alternate function 3>*/
#define GPIOx_AF_4  				4												/*< GPIO Alternate function 4>*/
#define GPIOx_AF_5  				5												/*< GPIO Alternate function 5>*/
#define GPIOx_AF_6  				6												/*< GPIO Alternate function 6>*/
#define GPIOx_AF_7  				7												/*< GPIO Alternate function 7>*/
#define GPIOx_AF_8  				8												/*< GPIO Alternate function 8>*/
#define GPIOx_AF_9  				9												/*< GPIO Alternate function 9>*/
#define GPIOx_AF_10  				10												/*< GPIO Alternate function 10>*/
#define GPIOx_AF_11  				11												/*< GPIO Alternate function 11>*/
#define GPIOx_AF_12  				12												/*< GPIO Alternate function 12>*/
#define GPIOx_AF_13  				13												/*< GPIO Alternate function 13>*/
#define GPIOx_AF_14  				14												/*< GPIO Alternate function 14>*/
#define GPIOx_AF_15  				15												/*< GPIO Alternate function 15>*/


/*
 * APIs
 */

// Clock
void GPIO_PCLKControl(GPIOx_Configuration_t *pGPIOx, uint8_t status);

//Init
void GPIO_Init(GPIOx_Handle_t *pGPIOx_Handle);
void GPIO_DeInit(GPIOx_Configuration_t *pGPIOx);

//Read
uint8_t GPIO_ReadPin(GPIOx_Configuration_t *pGPIOx, uint8_t pinNumber);
uint16_t GPIO_ReadPort(GPIOx_Configuration_t *pGPIOx);

//Write
void GPIO_WritePort(GPIOx_Configuration_t *pGPIOx, uint16_t data);
void GPIO_WritePin(GPIOx_Configuration_t *pGPIOx, uint8_t pinNumber, uint8_t data);

//Toggle
void GPIO_TogglePin(GPIOx_Configuration_t *pGPIOx, uint8_t pinNumber);

//IRQ
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t state);
void GPIO_IRQHandle(uint8_t pinNumber);



























