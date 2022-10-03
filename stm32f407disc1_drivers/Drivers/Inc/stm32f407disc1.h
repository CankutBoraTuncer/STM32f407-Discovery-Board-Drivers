

#include <stdint.h>
#include "stddef.h"

#ifndef INC_STM32F407DISC1_H_
#define INC_STM32F407DISC1_H_


/*
 * MISC
 */

#define __vol volatile
#define __weak __attribute__((weak))

#define ENABLE 1
#define DISABLE 0

#define SET ENABLE
#define RST DISABLE

#define GPIO_PIN_SET SET
#define GPIO_PIN_RST RST

#define DISC_IRQPRIORITY_1												1
#define DISC_IRQPRIORITY_2												2
#define DISC_IRQPRIORITY_3												3
#define DISC_IRQPRIORITY_4												4
#define DISC_IRQPRIORITY_5												5
#define DISC_IRQPRIORITY_6												6
#define DISC_IRQPRIORITY_7												7
#define DISC_IRQPRIORITY_8												8
#define DISC_IRQPRIORITY_9												9
#define DISC_IRQPRIORITY_10												10
#define DISC_IRQPRIORITY_11												11
#define DISC_IRQPRIORITY_12												12
#define DISC_IRQPRIORITY_13												13
#define DISC_IRQPRIORITY_14												14
#define DISC_IRQPRIORITY_15												15


#define GPIO_TO_NUMBER(x)											  ( (x == DISC_GPIOA) ? 0:\
																		(x == DISC_GPIOB) ? 1:\
																		(x == DISC_GPIOC) ? 2:\
																		(x == DISC_GPIOD) ? 3:\
																		(x == DISC_GPIOE) ? 4:\
																		(x == DISC_GPIOF) ? 5:\
																		(x == DISC_GPIOG) ? 6:\
																		(x == DISC_GPIOH) ? 7:\
																		(x == DISC_GPIOI) ? 8:\
																		(x == DISC_GPIOJ) ? 9:\
																		(x == DISC_GPIOK) ? 10:0 )




/*
 * CHIP SPECIFIC
 */

#define NVIC_ISER0          ( (__vol uint32_t*)0xE000E100 )
#define NVIC_ISER1          ( (__vol uint32_t*)0xE000E104 )
#define NVIC_ISER2          ( (__vol uint32_t*)0xE000E108 )
#define NVIC_ISER3          ( (__vol uint32_t*)0xE000E10c )


#define NVIC_ICER0 			((__vol uint32_t*)0XE000E180)
#define NVIC_ICER1			((__vol uint32_t*)0XE000E184)
#define NVIC_ICER2  		((__vol uint32_t*)0XE000E188)
#define NVIC_ICER3			((__vol uint32_t*)0XE000E18C)

#define NVIC_PR_BASE_ADDR 	((__vol uint32_t*)0xE000E400)

/*
 * MEMORY ADDRESS
 */

#define DISC_FLASH_BASEADDR												0x08000000U                                     /*< FLASH MEMORY(MAIN MEMMORY) BASE ADDRESS >*/
#define DISC_SRAM_BASEADDR												0x20000000U                                     /*< SRAM BASE ADDRESS >*/
#define DISC_SRAM2_BASEADDR												0x2001C000U		                                /*< SRAM2 BASE ADDRESS >*/
#define DISC_SYSTEMMEMORY_BASEADDR										0x1FFF0000U        						        /*< SYSTEM MEMORY(ROM) BASE ADDRESS >*/

#define DISC_SRAM 														DISC_SRAM_BASEADDR

/*
 * PERIPHERAL ADDRESS
 */

#define DISC_PERIPHERAL_BASEADDR										0x40000000U          						   /*< PERIPHERAL BASE ADDRESS >*/
#define DISC_APB1_BASEADDR											    0x40000000U          						   /*< APB1 BASE ADDRESS >*/
#define DISC_APB2_BASEADDR												0x40010000U         						   /*< APB2 BASE ADDRESS >*/
#define DISC_AHB1_BASEADDR												0x40020000U         						   /*< AHB1 BASE ADDRESS >*/
#define DISC_AHB2_BASEADDR												0x50000000U        						       /*< AHB2 BASE ADDRESS >*/
#define DISC_AHB3_BASEADDR												0xA0000000U                                    /*< AHB3 BASE ADDRESS >*/
/*
 * AHB1
 */

#define DISC_GPIOA_BASEADDR												(DISC_AHB1_BASEADDR + 0x0000U)			       /*< GPIOA BASE ADDRESS >*/
#define DISC_GPIOB_BASEADDR											    (DISC_AHB1_BASEADDR + 0x0400U)                  /*< GPIOB BASE ADDRESS >*/
#define DISC_GPIOC_BASEADDR												(DISC_AHB1_BASEADDR + 0x0800U)                  /*< GPIOC BASE ADDRESS >*/
#define DISC_GPIOD_BASEADDR												(DISC_AHB1_BASEADDR + 0x0C00U)                  /*< GPIOD BASE ADDRESS >*/
#define DISC_GPIOE_BASEADDR												(DISC_AHB1_BASEADDR + 0x01000U)                 /*< GPIOE BASE ADDRESS >*/
#define DISC_GPIOF_BASEADDR												(DISC_AHB1_BASEADDR + 0x01400U)                 /*< GPIOF BASE ADDRESS >*/
#define DISC_GPIOG_BASEADDR											    (DISC_AHB1_BASEADDR + 0x01800U)                 /*< GPIOG BASE ADDRESS >*/
#define DISC_GPIOH_BASEADDR												(DISC_AHB1_BASEADDR + 0x1C00U)                 /*< GPIOH BASE ADDRESS >*/
#define DISC_GPIOI_BASEADDR												(DISC_AHB1_BASEADDR + 0x2000U)                 /*< GPIOI BASE ADDRESS >*/
#define DISC_GPIOJ_BASEADDR												(DISC_AHB1_BASEADDR + 0x2400U)                 /*< GPIOJ BASE ADDRESS >*/
#define DISC_GPIOK_BASEADDR												(DISC_AHB1_BASEADDR + 0x2800U)                 /*< GPIOK BASE ADDRESS >*/

#define DISC_CRC_BASEADDR												(DISC_AHB1_BASEADDR + 0x3000U)			       /*< CRC BASE ADDRESS >*/
#define DISC_RCC_BASEADDR											    (DISC_AHB1_BASEADDR + 0x3800U)                 /*< RCC BASE ADDRESS >*/
#define DISC_FIR_BASEADDR												(DISC_AHB1_BASEADDR + 0x3C00U)                 /*< FLASH INTERFACE REGISTER BASE ADDRESS >*/
#define DISC_BKPSRAM_BASEADDR											(DISC_AHB1_BASEADDR + 0x4000U)                 /*< BKPSRAM BASE ADDRESS >*/
#define DISC_DMA1_BASEADDR												(DISC_AHB1_BASEADDR + 0x6000U)                 /*< DMA1 BASE ADDRESS >*/
#define DISC_DMA2_BASEADDR												(DISC_AHB1_BASEADDR + 0x6400U)                 /*< DMA2 BASE ADDRESS >*/
#define DISC_ETHERNET_BASEADDR											(DISC_AHB1_BASEADDR + 0x8000U)                 /*< ETHERNET MAC BASE ADDRESS >*/
#define DISC_DMA2D_BASEADDR										        (DISC_AHB1_BASEADDR + 0xB000U)                 /*< DMA2D BASE ADDRESS >*/
#define DISC_USBOTGHS_BASEADDR										    (DISC_AHB1_BASEADDR + 0x20000U)                /*< USB OTG HS BASE ADDRESS >*/

/*
 * AHB2
 */

#define DISC_USBOTGFS_BASEADDR										    (DISC_AHB2_BASEADDR + 0x000U)			       /*< USB OTG FS BASE ADDRESS >*/
#define DISC_DCMI_BASEADDR											    (DISC_AHB2_BASEADDR + 0x50000U)                /*< DCMI BASE ADDRESS >*/
//#define DISC_CRYP_BASEADDR												(DISC_AHB2_BASEADDR + 0x60000U)                /*< CRYP BASE ADDRESS >*/ NOT IMPLEMENTED IN THIS BOARD
//#define DISC_HASH_BASEADDR												(DISC_AHB2_BASEADDR + 0x60400U)                /*< HASH BASE ADDRESS >*/ NOT IMPLEMENTED IN THIS BOARD
#define DISC_RNG_BASEADDR												(DISC_AHB2_BASEADDR + 0x60800U)                /*< RNG BASE ADDRESS >*/

/*
 * APB1
 */

#define DISC_TIM2_BASEADDR												(DISC_APB1_BASEADDR + 0x000U)			          /*< TIM2 BASE ADDRESS >*/
#define DISC_TIM3_BASEADDR											    (DISC_APB1_BASEADDR + 0x400U)                     /*< TIM3 BASE ADDRESS >*/
#define DISC_TIM4_BASEADDR												(DISC_APB1_BASEADDR + 0x800U)                     /*< TIM4 BASE ADDRESS >*/
#define DISC_TIM5_BASEADDR												(DISC_APB1_BASEADDR + 0xC00U)                     /*< TIM5 BASE ADDRESS >*/
#define DISC_TIM6_BASEADDR												(DISC_APB1_BASEADDR + 0x1000U)                    /*< TIM6 BASE ADDRESS >*/
#define DISC_TIM7_BASEADDR												(DISC_APB1_BASEADDR + 0x1400U)                    /*< TIM7 BASE ADDRESS >*/
#define DISC_TIM12_BASEADDR											    (DISC_APB1_BASEADDR + 0x1800U)                    /*< TIM12 BASE ADDRESS >*/
#define DISC_TIM13_BASEADDR												(DISC_APB1_BASEADDR + 0x1C00U)                    /*< TIM13 BASE ADDRESS >*/
#define DISC_TIM14_BASEADDR												(DISC_APB1_BASEADDR + 0x2000U)                    /*< TIM14 BASE ADDRESS >*/
#define DISC_RTCBKP_BASEADDR											(DISC_APB1_BASEADDR + 0x2800U)                    /*< RTC & BKP REGISTERS BASE ADDRESS >*/
#define DISC_WWDG_BASEADDR												(DISC_APB1_BASEADDR + 0x2C00U)                    /*< WWDG BASE ADDRESS >*/
#define DISC_IWDG_BASEADDR												(DISC_APB1_BASEADDR + 0x3000U)			          /*< IWDG BASE ADDRESS >*/
#define DISC_I2S2EXT_BASEADDR											(DISC_APB1_BASEADDR + 0x3400U)                    /*< I2S2EXT BASE ADDRESS >*/
#define DISC_SPI2I2S2_BASEADDR											(DISC_APB1_BASEADDR + 0x3800U)                    /*< SPI2 / I2S2 INTERFACE REGISTER BASE ADDRESS >*/
#define DISC_SPI3I2S3_BASEADDR											(DISC_APB1_BASEADDR + 0x3C00U)                    /*< SPI3 / I2S3 BASE ADDRESS >*/
#define DISC_I2S3EXT_BASEADDR											(DISC_APB1_BASEADDR + 0x4000U)                    /*< I2S3EXT BASE ADDRESS >*/
#define DISC_USART2_BASEADDR											(DISC_APB1_BASEADDR + 0x4400U)                    /*< USART2 BASE ADDRESS >*/
#define DISC_USART3_BASEADDR											(DISC_APB1_BASEADDR + 0x4800U)                    /*< USART3 MAC BASE ADDRESS >*/
#define DISC_UART4_BASEADDR										        (DISC_APB1_BASEADDR + 0x4C00U)                    /*< UART4 BASE ADDRESS >*/
#define DISC_UART5_BASEADDR										        (DISC_APB1_BASEADDR + 0x5000U)                    /*< UART5 BASE ADDRESS >*/
#define DISC_I2C1_BASEADDR												(DISC_APB1_BASEADDR + 0x5400U)                    /*< I2C1 BASE ADDRESS >*/
#define DISC_I2C2_BASEADDR												(DISC_APB1_BASEADDR + 0x5800U)                    /*< I2C2 BASE ADDRESS >*/
#define DISC_I2C3_BASEADDR												(DISC_APB1_BASEADDR + 0x5C00U)			          /*< I2C3 BASE ADDRESS >*/
#define DISC_CAN1_BASEADDR											    (DISC_APB1_BASEADDR + 0x6400U)                    /*< CAN1 BASE ADDRESS >*/
#define DISC_CAN2_BASEADDR											    (DISC_APB1_BASEADDR + 0x6800U)                    /*< CAN2 INTERFACE REGISTER BASE ADDRESS >*/
#define DISC_PWR_BASEADDR											    (DISC_APB1_BASEADDR + 0x7000U)                    /*< PWR BASE ADDRESS >*/
#define DISC_DAC_BASEADDR											 	(DISC_APB1_BASEADDR + 0x7400U)                    /*< DAC BASE ADDRESS >*/
#define DISC_UART7_BASEADDR												(DISC_APB1_BASEADDR + 0x7800U)                    /*< UART7 BASE ADDRESS >*/
#define DISC_UART8_BASEADDR											    (DISC_APB1_BASEADDR + 0x7C00U)                    /*< UART8 MAC BASE ADDRESS >*/

/*
 * APB2
 */

#define DISC_TIM1_BASEADDR												(DISC_APB2_BASEADDR + 0x000U)			          /*< TIM1 BASE ADDRESS >*/
#define DISC_TIM8_BASEADDR											    (DISC_APB2_BASEADDR + 0x400U)                     /*< TIM8 BASE ADDRESS >*/
#define DISC_USART1_BASEADDR											(DISC_APB2_BASEADDR + 0x1000U)                    /*< USART1 BASE ADDRESS >*/
#define DISC_USART6_BASEADDR											(DISC_APB2_BASEADDR + 0x1400U)                    /*< USART6 BASE ADDRESS >*/
#define DISC_ADC123_BASEADDR											(DISC_APB2_BASEADDR + 0x2000U)                    /*< ADC1 & ADC2 & ADC3 BASE ADDRESS >*/
#define DISC_SDIO_BASEADDR												(DISC_APB2_BASEADDR + 0x2C00U)                    /*< SDIO BASE ADDRESS >*/
#define DISC_SPI1_BASEADDR											    (DISC_APB2_BASEADDR + 0x3000U)                    /*< SPI1 BASE ADDRESS >*/
#define DISC_SPI4_BASEADDR												(DISC_APB2_BASEADDR + 0x3400U)                    /*< SPI4 BASE ADDRESS >*/
#define DISC_SYSCFG_BASEADDR											(DISC_APB2_BASEADDR + 0x3800U)                    /*< SYSCFG BASE ADDRESS >*/
#define DISC_EXTI_BASEADDR												(DISC_APB2_BASEADDR + 0x3C00U)                    /*< EXTI REGISTERS BASE ADDRESS >*/
#define DISC_TIM9_BASEADDR												(DISC_APB2_BASEADDR + 0x4000U)                    /*< TIM9 BASE ADDRESS >*/
#define DISC_TIM10_BASEADDR												(DISC_APB2_BASEADDR + 0x4400U)			          /*< TIM10 BASE ADDRESS >*/
#define DISC_TIM11_BASEADDR												(DISC_APB2_BASEADDR + 0x4800U)                    /*< TIM11 BASE ADDRESS >*/
#define DISC_SPI5_BASEADDR												(DISC_APB2_BASEADDR + 0x5000U)                    /*< SPI5 INTERFACE REGISTER BASE ADDRESS >*/
#define DISC_SPI6_BASEADDR												(DISC_APB2_BASEADDR + 0x5400U)                    /*< SPI6 BASE ADDRESS >*/
#define DISC_SAI1_BASEADDR												(DISC_APB2_BASEADDR + 0x5800U)                    /*< SAI1 BASE ADDRESS >*/
#define DISC_LCDTFT_BASEADDR											(DISC_APB2_BASEADDR + 0x6800U)                    /*< LCDTFT BASE ADDRESS >*/

/*
 * IRQ
 */

#define DISC_WWDG_IRQ													0
#define DISC_PVD_IRQ													1
#define DISC_TAMP_STAMP_IRQ												2
#define DISC_RTC_WKUP_IRQ 												3
#define DISC_FLASH_IRQ 													4
#define DISC_RCC_IRQ 													5
#define DISC_EXTIx_IRQ(number)											(number + 6)
#define DISC_DMAx_Stream_IRQ											[[11, 12, 13, 14, 15, 16, 17, 47],[56, 57, 58, 59, 60, 68, 69, 70]]
#define DISC_ADC_IRQ 													18
#define DISC_CANx_x_IRQ													[[19,20,21,22],[70, 71, 72, 73]]
#define DISC_EXTI95_IRQ 												23
#define DISC_TIM1_BRK_TIM9_IRQ 											24
#define DISC_TIM1_UP_TIM10_IRQ 											25
#define DISC_TIM1_TRG_COM_TIM11_IRQ 									26
#define DISC_TIM1_CC_IRQ 												27
#define DISC_TIM24_IRQ 													[28, 29, 30]
#define DISC_I2Cx_IRQ 													[[31, 32], [33, 34], [72, 73]] // I2C 1 I2C 2 EVENT ERROR
#define DISC_SPIx_IRQ(number)											((number == 1) ? 35 : (number == 2) ? 36 : (number == 3) ? 51: -1)
#define DISC_USARTx_IRQ 												[37, 38, 39, 71]
#define DISC_EXTI1510_IRQ												40
#define DISC_RTC_Alarm_IRQ 												41
#define DISC_OTG_FS_WKUP_IRQ 											42
#define DISC_TIM8_BRK_TIM12_IRQ 										43
#define DISC_TIM8_UP_TIM13_IRQ 											44
#define DISC_TIM8_TRG_COM_TIM14_IRQ 									45
#define DISC_TIM8_CC_IRQ 												46
#define DISC_FSMC_IRQ 													48
#define DISC_SDIO_IRQ 													49
#define DISC_TIM5_IRQ 													50
#define DISC_UART45_IRQ 												[52, 53]
#define DISC_TIM6_DAC_IRQ 												54
#define DISC_TIM7_IRQ 													55
#define DISC_ETH_IRQ 													61
#define DISC_ETH_WKUP_IRQ 												62
#define DISC_OTG_FS_IRQ	 												67
#define DISC_OTG_HS_EP1_OUT_IRQ 										74
#define DISC_OTG_HS_EP1_IN_IRQ 											75
#define DISC_OTG_HS_WKUP_IRQ 											76
#define DISC_OTG_HS_IRQ 												77
#define DISC_DCMI_IRQ 													78
#define DISC_CRYP_IRQ 													79
#define DISC_HASH_RNG_IRQ 												80
#define DISC_FPU_IRQ 													81


/*
 * Configuration Registers
 */

typedef struct{
	__vol uint32_t GPIO_MODER;                                                /*< GPIO port mode register>*/
	__vol uint32_t GPIO_OTYPER;                                               /*< GPIO port output type register>*/
	__vol uint32_t GPIO_OSPEEDR;                                              /*< GPIO port output speed register>*/
	__vol uint32_t GPIO_PUPDR;                                                /*< GPIO port pull-up/pull-down register>*/
	__vol uint32_t GPIO_IDR;                                                  /*< GPIO port input data register>*/
	__vol uint32_t GPIO_ODR;                                                  /*< GPIO port output data register>*/
	__vol uint32_t GPIO_BSRR;                                                 /*< GPIO port bit set/reset register>*/
	__vol uint32_t GPIO_LCKR;                                                 /*< GPIO port configuration lock register>*/
	__vol uint32_t GPIO_AFR[1];                                                 /*< GPIO alternate function low register[0] GPIO alternate function high register[1]>*/
}GPIOx_Configuration_t;

#define DISC_GPIOA 														 ((GPIOx_Configuration_t*) DISC_GPIOA_BASEADDR)   /*< GPIOA CONFIG MACRO >*/

#define DISC_GPIOB 														 ((GPIOx_Configuration_t*) DISC_GPIOB_BASEADDR)   /*< GPIOB CONFIG MACRO >*/

#define DISC_GPIOC 														 ((GPIOx_Configuration_t*) DISC_GPIOC_BASEADDR)   /*< GPIOC CONFIG MACRO >*/

#define DISC_GPIOD 														 ((GPIOx_Configuration_t*) DISC_GPIOD_BASEADDR)   /*< GPIOD CONFIG MACRO >*/

#define DISC_GPIOE 														 ((GPIOx_Configuration_t*) DISC_GPIOE_BASEADDR)   /*< GPIOE CONFIG MACRO >*/

#define DISC_GPIOF 														 ((GPIOx_Configuration_t*) DISC_GPIOF_BASEADDR)   /*< GPIOF CONFIG MACRO >*/

#define DISC_GPIOG 														 ((GPIOx_Configuration_t*) DISC_GPIOG_BASEADDR)   /*< GPIOG CONFIG MACRO >*/

#define DISC_GPIOH 														 ((GPIOx_Configuration_t*) DISC_GPIOH_BASEADDR)   /*< GPIOH CONFIG MACRO >*/

#define DISC_GPIOI 														 ((GPIOx_Configuration_t*) DISC_GPIOI_BASEADDR)   /*< GPIOI CONFIG MACRO >*/

#define DISC_GPIOJ 														 ((GPIOx_Configuration_t*) DISC_GPIOJ_BASEADDR)   /*< GPIOJ CONFIG MACRO >*/

#define DISC_GPIOK 														 ((GPIOx_Configuration_t*) DISC_GPIOK_BASEADDR)   /*< GPIOK CONFIG MACRO >*/


typedef struct{
	__vol uint32_t CRC_DR;                                                /*< Data register>*/
	__vol uint32_t CRC_IDR;                                               /*< Independent data register>*/
	__vol uint32_t CRC_CR;                                                /*< Control register>*/
}CRC_Configuration_t;

#define DISC_CRC 														 ((CRC_Configuration_t*) DISC_CRC_BASEADDR)   /*< CRC CONFIG MACRO >*/


typedef struct{
	__vol uint32_t RCC_CR;                                                /*< RCC clock control register>*/
	__vol uint32_t RCC_PLLCFGR;                                           /*< RCC PLL configuration register>*/
	__vol uint32_t RCC_CFGR;                                              /*< RCC clock configuration register>*/
	__vol uint32_t RCC_CIR;                                               /*< RCC clock interrupt register>*/
	__vol uint32_t RCC_AHB1RSTR;                                          /*< RCC AHB1 peripheral reset register>*/
	__vol uint32_t RCC_AHB2RSTR;                                          /*< RCC AHB2 peripheral reset register>*/
	__vol uint32_t RCC_AHB3RSTR;                                          /*< RCC AHB3 peripheral reset register>*/
		  uint32_t RCC_RESERVED1; 									  /*< RESERVED >*/
	__vol uint32_t RCC_APB1RSTR;                                          /*< RCC APB1 peripheral reset register>*/
	__vol uint32_t RCC_APB2RSTR;                                          /*< RCC APB2 peripheral reset register >*/
	      uint32_t RCC_RESERVED2[2]; 									  /*< RESERVED -0x2C >*/
	__vol uint32_t RCC_AHB1ENR;                                           /*< RCC AHB1 peripheral clock enable register >*/
	__vol uint32_t RCC_AHB2ENR;                                           /*< RCC AHB2 peripheral clock enable register >*/
	__vol uint32_t RCC_AHB3ENR;                                           /*< RCC AHB3 peripheral clock enable register >*/
	      uint32_t RCC_RESERVED3; 									  /*< RESERVED >*/
	__vol uint32_t RCC_APB1ENR;                                           /*< RCC APB1 peripheral clock enable register >*/
	__vol uint32_t RCC_APB2ENR;                                           /*< RCC APB2 peripheral clock enable register >*/
	      uint32_t RCC_RESERVED4[2]; 									  /*< RESERVED >*/
	__vol uint32_t RCC_AHB1LPENR;                                         /*< RCC AHB1 peripheral clock enable in low power mode register >*/
	__vol uint32_t RCC_AHB2LPENR;                                         /*< RCC AHB2 peripheral clock enable in low power mode register >*/
	__vol uint32_t RCC_AHB3LPENR;                                         /*< RCC APB3 peripheral clock enable in low power mode register >*/
	      uint32_t RCC_RESERVED5; 									  /*< RESERVED >*/
	__vol uint32_t RCC_APB1LPENR;                                         /*< RCC APB1 peripheral clock enabled in low power mode register >*/
	__vol uint32_t RCC_APB2LPENR;                                         /*< RCC APB1 peripheral clock enabled in low power mode register >*/
	      uint32_t RCC_RESERVED6[2]; 									  /*< RESERVED >*/
	__vol uint32_t RCC_BDCR;                                              /*< RCC Backup domain control register >*/
	__vol uint32_t RCC_CSR;                                               /*< RCC clock control & status register >*/
	      uint32_t RCC_RESERVED7[2]; 									  /*< RESERVED >*/
	__vol uint32_t RCC_SSCGR;                                             /*< RCC spread spectrum clock generation register >*/
	__vol uint32_t RCC_PLLI2SCFGR;                                        /*< RCC PLLI2S configuration register >*/
	__vol uint32_t RCC_PLLSAICFGR;                                        /*< RCC PLL configuration register >*/
	__vol uint32_t RCC_DCKCFGR;                                       	  /*< RCC Dedicated Clock Configuration Register >*/
}RCC_Configuration_t;

#define DISC_RCC 			 											 ((RCC_Configuration_t*) DISC_RCC_BASEADDR)   /*< RCC CONFIG MACRO >*/


typedef struct{
	__vol uint32_t FLASH_ACR;                                               /*< Flash access control register >*/
	__vol uint32_t FLASH_KEYR;                                              /*< Flash key register >*/
	__vol uint32_t FLASH_OPTKEYR;                                           /*< Flash option key register >*/
	__vol uint32_t FLASH_SR;                                                /*< Flash status register >*/
	__vol uint32_t FLASH_CR;                                                /*< Flash control register >*/
	__vol uint32_t FLASH_OPTCR;                                             /*< Flash option control register >*/
}FIR_Configuration_t;

#define DISC_FIR														 ((FIR_Configuration_t*) DISC_FIR_BASEADDR)   /*< FIR CONFIG MACRO >*/

typedef struct{
	__vol uint32_t DMA_LISR;                                                /*< DMA low interrupt status register >*/
	__vol uint32_t DMA_HISR;                                                /*< DMA high interrupt status register >*/
	__vol uint32_t DMA_LIFCR;                                               /*< DMA low interrupt flag clear register >*/
	__vol uint32_t DMA_HIFCR;                                               /*< DMA high interrupt flag clear register >*/
	__vol uint32_t DMA_S0CR;                                                /*< DMA stream 0 configuration register >*/
	__vol uint32_t DMA_S0NDTR;                                              /*< DMA stream 0 number of data register >*/
	__vol uint32_t DMA_S0PAR;                                               /*< DMA stream 0 peripheral address register  >*/
	__vol uint32_t DMA_S0M0AR; 										        /*< DMA stream 0 memory 0 address register -  1C >*/
	__vol uint32_t DMA_S0M1AR;                                              /*< DMA stream 0 memory 1 address register  >*/
	__vol uint32_t DMA_S0FCR;                                               /*< DMA stream 0 FIFO control register  >*/
	__vol uint32_t DMA_S1CR;                                                /*< DMA stream 1 configuration register  >*/
	__vol uint32_t DMA_S1NDTR;                                              /*< DMA stream 1 number of data register  >*/
	__vol uint32_t DMA_S1PAR;                                               /*< DMA stream 1 peripheral address register  >*/
	__vol uint32_t DMA_S1M0AR; 										        /*< DMA stream 1 memory 0 address register  >*/
	__vol uint32_t DMA_S1M1AR;                                              /*< DMA stream 1 memory 1 address register  >*/
	__vol uint32_t DMA_S1FCR;                                               /*< DMA stream 1 FIFO control register  >*/
	__vol uint32_t DMA_S2CR;                                                /*< DMA stream 2 configuration register  >*/
	__vol uint32_t DMA_S2NDTR;                                              /*< DMA stream 2 number of data register  >*/
	__vol uint32_t DMA_S2PAR;                                               /*< DMA stream 2 peripheral address register  >*/
	__vol uint32_t DMA_S2M0AR; 										        /*< DMA stream 2 memory 0 address register  >*/
	__vol uint32_t DMA_S2M1AR;                                              /*< DMA stream 2 memory 1 address register  >*/
	__vol uint32_t DMA_S2FCR;                                               /*< DMA stream 2 FIFO control register  >*/
	__vol uint32_t DMA_S3CR;                                                /*< DMA stream 3 configuration register  >*/
	__vol uint32_t DMA_S3NDTR;                                              /*< DMA stream 3 number of data register  >*/
	__vol uint32_t DMA_S3PAR;                                               /*< DMA stream 3 peripheral address register  >*/
	__vol uint32_t DMA_S3M0AR; 										        /*< DMA stream 3 memory 0 address register  >*/
	__vol uint32_t DMA_S3M1AR;                                              /*< DMA stream 3 memory 1 address register  >*/
	__vol uint32_t DMA_S3FCR;                                               /*< DMA stream 3 FIFO control register  >*/
	__vol uint32_t DMA_S4CR;                                                /*< DMA stream 4 configuration register  >*/
	__vol uint32_t DMA_S4NDTR;                                              /*< DMA stream 4 number of data register  >*/
	__vol uint32_t DMA_S4PAR;                                               /*< DMA stream 4 peripheral address register  >*/
	__vol uint32_t DMA_S4M0AR; 										        /*< DMA stream 4 memory 0 address register  >*/
	__vol uint32_t DMA_S4M1AR;                                              /*< DMA stream 4 memory 1 address register  >*/
	__vol uint32_t DMA_S4FCR;                                               /*< DMA stream 4 FIFO control register  >*/
	__vol uint32_t DMA_S5CR;                                                /*< DMA stream 5 configuration register  >*/
	__vol uint32_t DMA_S5NDTR;                                              /*< DMA stream 5 number of data register  >*/
	__vol uint32_t DMA_S5PAR;                                               /*< DMA stream 5 peripheral address register  >*/
	__vol uint32_t DMA_S5M0AR; 										        /*< DMA stream 5 memory 0 address register  >*/
	__vol uint32_t DMA_S5M1AR;                                              /*< DMA stream 5 memory 1 address register  >*/
	__vol uint32_t DMA_S5FCR;                                               /*< DMA stream 5 FIFO control register  >*/
	__vol uint32_t DMA_S6CR;                                                /*< DMA stream 6 configuration register  >*/
	__vol uint32_t DMA_S6NDTR;                                              /*< DMA stream 6 number of data register  >*/
	__vol uint32_t DMA_S6PAR;                                               /*< DMA stream 6 peripheral address register  >*/
	__vol uint32_t DMA_S6M0AR; 										        /*< DMA stream 6 memory 0 address register  >*/
	__vol uint32_t DMA_S6M1AR;                                              /*< DMA stream 6 memory 1 address register  >*/
	__vol uint32_t DMA_S6FCR;                                               /*< DMA stream 6 FIFO control register  >*/
	__vol uint32_t DMA_S7CR;                                                /*< DMA stream 7 configuration register  >*/
	__vol uint32_t DMA_S7NDTR;                                              /*< DMA stream 7 number of data register  >*/
	__vol uint32_t DMA_S7PAR;                                               /*< DMA stream 7 peripheral address register  >*/
	__vol uint32_t DMA_S7M0AR; 										        /*< DMA stream 7 memory 0 address register  >*/
	__vol uint32_t DMA_S7M1AR;                                              /*< DMA stream 7 memory 1 address register  >*/
	__vol uint32_t DMA_S7FCR;                                               /*< DMA stream 7 FIFO control register  >*/
}DMAx_Configuration_t;

#define DISC_DMA1														 ((DMAx_Configuration_t*) DISC_DMA1_BASEADDR)   /*< DMA1 CONFIG MACRO >*/

#define DISC_DMA2														 ((DMAx_Configuration_t*) DISC_DMA2_BASEADDR)   /*< DMA2 CONFIG MACRO >*/


typedef struct{
	__vol uint32_t ETH_MACCR;                                               /*< Ethernet MAC configuration register  >*/
	__vol uint32_t ETH_MACFFR;                                              /*< Ethernet MAC frame filter register  >*/
	__vol uint32_t ETH_MACHTHR;                                             /*< Ethernet MAC hash table high register  >*/
	__vol uint32_t ETH_MACHTLR;                                             /*< Ethernet MAC hash table low register  >*/
	__vol uint32_t ETH_MACMIIAR;                                            /*< Ethernet MAC MII address register >*/
	__vol uint32_t ETH_MACMIIDR;                                            /*< Ethernet MAC MII data register  >*/
	__vol uint32_t ETH_MACFCR;                                              /*< Ethernet MAC flow control register  >*/
	__vol uint32_t ETH_MACVLANTR; 										    /*< Ethernet MAC flow control register  >*/
	__vol uint32_t ETH_MACRWUFFR;                                           /*< Ethernet MAC remote wake-up frame filter register  >*/
	__vol uint32_t ETH_MACPMTCSR;                                           /*< Ethernet MAC PMT control and status register  >*/
	__vol uint32_t ETH_MACDBGR;                                             /*< Ethernet MAC debug register  >*/
	__vol uint32_t ETH_MACSR;                                               /*< Ethernet MAC interrupt status register >*/
	__vol uint32_t ETH_MACIMR;                                              /*< Ethernet MAC interrupt mask register  >*/
	__vol uint32_t ETH_MACA0HR; 										    /*< Ethernet MAC address 0 high register  >*/
	__vol uint32_t ETH_MACA0LR;                                             /*< Ethernet MAC address 0 low register  >*/
	__vol uint32_t ETH_MACA1HR;                                             /*< Ethernet MAC address 1 high register  >*/
	__vol uint32_t ETH_MACA1LR;                                             /*< Ethernet MAC address1 low register  >*/
	__vol uint32_t ETH_MACA2HR;                                             /*< Ethernet MAC address 2 high register  >*/
	__vol uint32_t ETH_MACA2LR;                                             /*< Ethernet MAC address 2 low register  >*/
	__vol uint32_t ETH_MACA3HR; 										    /*< Ethernet MAC address 3 high register  >*/
	__vol uint32_t ETH_MACA3LR;                                             /*< Ethernet MAC address 3 low register  >*/
	__vol uint32_t ETH_MMCCR;                                               /*< Ethernet MMC control register  >*/
	__vol uint32_t ETH_MMCRIR;                                              /*< Ethernet MMC receive interrupt register  >*/
	__vol uint32_t ETH_MMCTIR;                                              /*< Ethernet MMC transmit interrupt register  >*/
	__vol uint32_t ETH_MMCRIMR;                                             /*< Ethernet MMC receive interrupt mask register  >*/
	__vol uint32_t ETH_MMCTIMR; 										    /*< Ethernet MMC transmit interrupt mask register  >*/
	__vol uint32_t ETH_MMCTGFSCCR;                                          /*< Ethernet MMC transmitted good frames after a single collision counter register  >*/
	__vol uint32_t ETH_MMCTGFMSCCR;                                         /*< Ethernet MMC transmitted good frames after more than a single collision counter register  >*/
	__vol uint32_t ETH_MMCTGFCR;                                            /*< Ethernet MMC transmitted good frames counter register  >*/
	__vol uint32_t ETH_MMCRFCECR;                                           /*< Ethernet MMC received frames with CRC error counter register  >*/
	__vol uint32_t ETH_MMCRFAECR;                                           /*< Ethernet MMC received frames with alignment error counter register  >*/
	__vol uint32_t ETH_MMCRGUFCR; 										    /*< Ethernet MMC received good unicast frames counter register  >*/
	__vol uint32_t ETH_PTPTSCR;                                             /*< Ethernet PTP time stamp control register  >*/
	__vol uint32_t ETH_PTPSSIR;                                             /*< Ethernet PTP subsecond increment register  >*/
	__vol uint32_t ETH_PTPTSHR;                                             /*< Ethernet PTP time stamp high register  >*/
	__vol uint32_t ETH_PTPTSLR;                                             /*< Ethernet PTP time stamp low register  >*/
	__vol uint32_t ETH_PTPTSHUR;                                            /*< Ethernet PTP time stamp high update register  >*/
	__vol uint32_t ETH_PTPTSLUR; 										    /*< Ethernet PTP time stamp low update register  >*/
	__vol uint32_t ETH_PTPTSAR;                                             /*< Ethernet PTP time stamp addend register  >*/
	__vol uint32_t ETH_PTPTTHR;                                             /*< Ethernet PTP target time high register  >*/
	__vol uint32_t ETH_PTPTTLR;                                             /*< Ethernet PTP target time low register  >*/
	__vol uint32_t ETH_PTPTSSR;                                             /*< Ethernet PTP time stamp status register  >*/
	__vol uint32_t ETH_PTPPPSCR;                                            /*< Ethernet PTP PPS control register  >*/
	__vol uint32_t ETH_DMABMR; 										        /*< Ethernet DMA bus mode register  >*/
	__vol uint32_t ETH_DMATPDR;                                             /*< Ethernet DMA transmit poll demand register  >*/
	__vol uint32_t ETH_DMARPDR;                                             /*< Ethernet DMA receive poll demand register  >*/
	__vol uint32_t ETH_DMARDLAR;                                            /*< Ethernet DMA receive descriptor list address register  >*/
	__vol uint32_t ETH_DMATDLAR;                                            /*< Ethernet DMA transmit descriptor list address register  >*/
	__vol uint32_t ETH_DMASR;                                               /*< Ethernet DMA status register  >*/
	__vol uint32_t ETH_DMAOMR; 										        /*< Ethernet DMA operation mode register  >*/
	__vol uint32_t ETH_DMAIER;                                              /*< Ethernet DMA interrupt enable register  >*/
	__vol uint32_t ETH_DMAMFBOCR;                                           /*< Ethernet DMA missed frame and buffer overflow counter register  >*/
	__vol uint32_t ETH_DMARSWTR; 										    /*< Ethernet DMA receive status watchdog timer register  >*/
	__vol uint32_t ETH_DMACHTDR;                                            /*< Ethernet DMA current host transmit descriptor register  >*/
	__vol uint32_t ETH_DMACHRDR;                                            /*< Ethernet DMA current host receive descriptor register  >*/
	__vol uint32_t ETH_DMACHTBAR;                                           /*< Ethernet DMA current host transmit buffer address register  >*/
	__vol uint32_t ETH_DMACHRBAR;                                           /*< Ethernet DMA current host receive buffer address register  >*/

}ETHERNET_Configuration_t;

#define DISC_ETHERNET													 ((ETHERNET_Configuration_t*) DISC_ETHERNET_BASEADDR)   /*< ETHERNET CONFIG MACRO >*/


typedef struct{
	__vol uint32_t DMA2D_CR;                                               /*< DMA2D control register  >*/
	__vol uint32_t DMA2D_ISR;                                              /*< DMA2D Interrupt Status Register  >*/
	__vol uint32_t DMA2D_IFCR;                                             /*< DMA2D interrupt flag clear register  >*/
	__vol uint32_t DMA2D_FGMAR;                                            /*< DMA2D foreground memory address register  >*/
	__vol uint32_t DMA2D_FGOR;                                             /*< DMA2D foreground offset register >*/
	__vol uint32_t DMA2D_BGMAR;                                            /*< DMA2D background memory address register  >*/
	__vol uint32_t DMA2D_BGOR;                                             /*< DMA2D background offset register  >*/
	__vol uint32_t DMA2D_FGPFCCR; 										   /*< DMA2D foreground PFC control register  >*/
	__vol uint32_t DMA2D_FGCOLR;                                           /*< DMA2D foreground color register  >*/
	__vol uint32_t DMA2D_BGPFCCR;                                          /*< DMA2D background PFC control register  >*/
	__vol uint32_t DMA2D_BGCOLR;                                           /*< DMA2D background color register  >*/
	__vol uint32_t DMA2D_FGCMAR;                                           /*< DMA2D foreground CLUT memory address register >*/
	__vol uint32_t DMA2D_BGCMAR;                                           /*< DMA2D background CLUT memory address register  >*/
	__vol uint32_t DMA2D_OPFCCR; 										   /*< DMA2D output PFC control register  >*/
	__vol uint32_t DMA2D_OCOLR;                                            /*< DMA2D output color register  >*/
	__vol uint32_t DMA2D_OMAR;                                             /*< DMA2D output memory address register  >*/
	__vol uint32_t DMA2D_OOR;                                              /*< DMA2D output offset register  >*/
	__vol uint32_t DMA2D_NLR;                                              /*< DMA2D number of line register  >*/
	__vol uint32_t DMA2D_LWR;                                              /*< DMA2D line watermark register  >*/
	__vol uint32_t DMA2D_AMTCR; 										   /*< DMA2D AHB master timer configuration register  >*/
	__vol uint32_t DMA2D_FGCLUT;                                           /*< ?? >*/
	__vol uint32_t DMA2D_BGCLUT;                                           /*< ??  >*/

}DMA2D_Configuration_t;

#define DISC_DMA2D													     ((DMA2D_Configuration_t*) DISC_DMA2D_BASEADDR)   /*< DMA2D CONFIG MACRO >*/


typedef struct{
	__vol uint32_t DCMI_CR;                                              /*< DCMI control register >*/
	__vol uint32_t DCMI_SR;                                              /*< DCMI status register >*/
	__vol uint32_t DCMI_RIS;                                             /*< DCMI raw interrupt status register >*/
	__vol uint32_t DCMI_IER;                                             /*< DCMI interrupt enable register >*/
	__vol uint32_t DCMI_MIS;                                             /*< DCMI masked interrupt status register >*/
	__vol uint32_t DCMI_ICR;                                             /*< DCMI interrupt clear register >*/
	__vol uint32_t DCMI_ESCR;                                            /*< DCMI embedded synchronization code register >*/
	__vol uint32_t DCMI_ESUR;                                            /*< DCMI embedded synchronization unmask register >*/
	__vol uint32_t DCMI_CWSTRT;                                          /*< DCMI crop window start >*/
	__vol uint32_t DCMI_CWSIZE;                                          /*< DCMI crop window size >*/
	__vol uint32_t DCMI_DR;                                              /*< DCMI data register >*/
}DCMI_Configuration_t;

#define DISC_DCMI														 ((DCMI_Configuration_t*) DISC_DCMI_BASEADDR)   /*< DCMI CONFIG MACRO >*/


typedef struct{
	__vol uint32_t RNG_CR;                                              /*< RNG control register >*/
	__vol uint32_t RNG_SR;                                              /*< RNG status register >*/
	__vol uint32_t RNG_DR;                                              /*< RNG data register >*/
}RNG_Configuration_t;

#define DISC_RNG														 ((RNG_Configuration_t*) DISC_RNG_BASEADDR)   /*< RNG CONFIG MACRO >*/


typedef struct{
	__vol uint32_t TIMx_CR1;                                              /*< TIMx control register >*/
	__vol uint32_t TIMx_CR2;                                              /*< TIMx control register 2 >*/
	__vol uint32_t TIMx_SMCR;                                             /*< TIMx slave mode control register >*/
	__vol uint32_t TIMx_DIER;                                             /*< TIMx DMA/Interrupt enable register >*/
	__vol uint32_t TIMx_SR;                                               /*< TIMx status register >*/
	__vol uint32_t TIMx_EGR;                                              /*< TIMx event generation register >*/
	__vol uint32_t TIMx_CCMR1;                                            /*< TIMx capture/compare mode register 1 >*/
	__vol uint32_t TIMx_CCMR2;                                            /*< TIMx capture/compare mode register 2 >*/
	__vol uint32_t TIMx_CCER;                                             /*< TIMx capture/compare enable register >*/
	__vol uint32_t TIMx_CNT;                                              /*< TIMx counter >*/
	__vol uint32_t TIMx_PSC;                                              /*< TIMx prescaler >*/
	__vol uint32_t TIMx_ARR;                                              /*< TIMx auto-reload register >*/
	__vol uint32_t TIMx_CCR1;                                             /*< TIMx capture/compare register 1 >*/
	__vol uint32_t TIMx_CCR2;                                             /*< TIMx capture/compare register 2 >*/
	__vol uint32_t TIMx_CCR3;                                             /*< TIMx capture/compare register 3 >*/
	__vol uint32_t TIMx_CCR4;                                             /*< TIMx capture/compare register 4 >*/
	      uint32_t TIMx_RESERVED;                                         /*< RESERVED >*/
	__vol uint32_t TIMx_DCR;                                              /*< TIMx DMA control register >*/
	__vol uint32_t TIMx_DMAR;                                             /*< TIMx DMA address for full transfer >*/
	__vol uint32_t TIM25_OR;                                              /*< TIM2 & TIM5 option register >*/

}TIM25_Configuration_t;

#define DISC_TIM2														 ((TIM25_Configuration_t*) DISC_TIM2_BASEADDR)   /*< TIM2 CONFIG MACRO >*/

#define DISC_TIM3														 ((TIM25_Configuration_t*) DISC_TIM3_BASEADDR)   /*< TIM3 CONFIG MACRO >*/

#define DISC_TIM4														 ((TIM25_Configuration_t*) DISC_TIM4_BASEADDR)   /*< TIM4 CONFIG MACRO >*/

#define DISC_TIM5														 ((TIM25_Configuration_t*) DISC_TIM5_BASEADDR)   /*< TIM5 CONFIG MACRO >*/


typedef struct{
	__vol uint32_t TIMx_CR1;                                              /*< TIMx control register >*/
	__vol uint32_t TIMx_CR2;                                              /*< TIMx control register 2 >*/
	__vol uint32_t TIMx_DIER;                                             /*< TIMx DMA/Interrupt enable register >*/
	__vol uint32_t TIMx_SR;                                               /*< TIMx status register >*/
	__vol uint32_t TIMx_EGR;                                              /*< TIMx event generation register >*/
	__vol uint32_t TIMx_RESERVED;                                         /*< RESERVED>*/
	__vol uint32_t TIMx_RESERVED2;                                        /*< RESERVED >*/
	__vol uint32_t TIMx_RESERVED3;                                        /*< RESERVED >*/
	__vol uint32_t TIMx_CNT;                                              /*< TIMx counter >*/
	__vol uint32_t TIMx_PSC;                                              /*< TIMx prescaler >*/
	__vol uint32_t TIMx_ARR;                                              /*< TIMx auto-reload register >*/

}TIM67_Configuration_t;

#define DISC_TIM6														 ((TIM67_Configuration_t*) DISC_TIM6_BASEADDR)   /*< TIM6 CONFIG MACRO >*/

#define DISC_TIM7														 ((TIM67_Configuration_t*) DISC_TIM7_BASEADDR)   /*< TIM7 CONFIG MACRO >*/


typedef struct{
	__vol uint32_t TIMx_CR1;                                              /*< TIMx control register >*/
	__vol uint32_t TIMx_SMCR;                                             /*< TIMx slave mode control register >*/
	__vol uint32_t TIMx_DIER;                                             /*< TIMx DMA/Interrupt enable register >*/
	__vol uint32_t TIMx_SR;                                               /*< TIMx status register >*/
	__vol uint32_t TIMx_EGR;                                              /*< TIMx event generation register >*/
	__vol uint32_t TIMx_CCMR1;                                            /*< TIMx capture/compare mode register 1 >*/
    	  uint32_t TIMx_RESERVED;                                         /*< RESERVED >*/
	__vol uint32_t TIMx_CCER;                                             /*< TIMx capture/compare enable register >*/
	__vol uint32_t TIMx_CNT;                                              /*< TIMx counter >*/
	__vol uint32_t TIMx_PSC;                                              /*< TIMx prescaler >*/
	__vol uint32_t TIMx_ARR;                                              /*< TIMx auto-reload register >*/
    	  uint32_t TIMx_RESERVED2;
	__vol uint32_t TIMx_CCR1;                                             /*< TIMx capture/compare register 1 >*/
	__vol uint32_t TIMx_CCR2;                                             /*< TIMx capture/compare register 2 >*/
	__vol uint32_t TIMx_CCR3;                                             /*< TIMx capture/compare register 3 >*/
	__vol uint32_t TIMx_CCR4;                                             /*< TIMx capture/compare register 4 >*/
	      uint32_t TIMx_RESERVED3[15];                                    /*< RESERVED >*/

}TIM912_Configuration_t;

#define DISC_TIM9														 ((TIM912_Configuration_t*) DISC_TIM9_BASEADDR)    /*< TIM9 CONFIG MACRO >*/

#define DISC_TIM12														 ((TIM912_Configuration_t*) DISC_TIM12_BASEADDR)   /*< TIM12 CONFIG MACRO >*/


typedef struct{
	__vol uint32_t TIMx_CR1;                                              /*< TIMx control register >*/
	__vol uint32_t TIMx_SMCR;                                             /*< TIMx slave mode control register >*/
	__vol uint32_t TIMx_DIER;                                             /*< TIMx DMA/Interrupt enable register >*/
	__vol uint32_t TIMx_SR;                                               /*< TIMx status register >*/
	__vol uint32_t TIMx_EGR;                                              /*< TIMx event generation register >*/
	__vol uint32_t TIMx_CCMR1;                                            /*< TIMx capture/compare mode register 1 >*/
	      uint32_t TIMx_RESERVED;                                         /*< RESERVED >*/
	__vol uint32_t TIMx_CCER;                                             /*< TIMx capture/compare enable register >*/
	__vol uint32_t TIMx_CNT;                                              /*< TIMx counter >*/
	__vol uint32_t TIMx_PSC;                                              /*< TIMx prescaler >*/
	__vol uint32_t TIMx_ARR;                                              /*< TIMx auto-reload register >*/
	      uint32_t TIMx_RESERVED2;                                        /*< RESERVED >*/
	__vol uint32_t TIMx_CCR1;                                             /*< TIMx capture/compare register 1 >*/
	      uint32_t TIMx_RESERVED3[19];                                    /*< RESERVED >*/
	__vol uint32_t TIM25_OR;                                              /*< TIM2 & TIM5 option register >*/

}TIM1014_Configuration_t;

#define DISC_TIM10														 ((TIM1014_Configuration_t*) DISC_TIM10_BASEADDR)   /*< TIM10 CONFIG MACRO >*/

#define DISC_TIM11														 ((TIM1014_Configuration_t*) DISC_TIM11_BASEADDR)   /*< TIM11 CONFIG MACRO >*/

#define DISC_TIM13														 ((TIM1014_Configuration_t*) DISC_TIM13_BASEADDR)   /*< TIM13 CONFIG MACRO >*/

#define DISC_TIM14														 ((TIM1014_Configuration_t*) DISC_TIM14_BASEADDR)   /*< TIM14 CONFIG MACRO >*/


typedef struct{
	__vol uint32_t RTC_TR;                                              /*< RTC time register >*/
	__vol uint32_t RTC_DR;                                              /*< RTC date register >*/
	__vol uint32_t RTC_CR;                                              /*< RTC control register >*/
	__vol uint32_t RTC_ISR;                                             /*< RTC initialization and status register >*/
	__vol uint32_t RTC_PRER;                                            /*< RTC prescaler register >*/
	__vol uint32_t RTC_WUTR;                                            /*< RTC wakeup timer register >*/
	__vol uint32_t RTC_CALIBR;                                          /*< RTC calibration register >*/
	__vol uint32_t RTC_ALRMAR;                                          /*< RTC alarm A register >*/
	__vol uint32_t RTC_ALRMBR;                                          /*< RTC alarm B register >*/
	__vol uint32_t RTC_WPR;                                             /*< RTC write protection register >*/
	__vol uint32_t RTC_SSR;                                             /*< RTC sub second register >*/
	__vol uint32_t RTC_SHIFTR;                                          /*< RTC shift control register >*/
	__vol uint32_t RTC_TSTR;                                            /*< RTC time stamp time register >*/
	__vol uint32_t RTC_TSDR;                                            /*< RTC time stamp date register >*/
	__vol uint32_t RTC_TSSSR;                                           /*< RTC timestamp sub second register >*/
	__vol uint32_t RTC_CALR;                                            /*< RTC calibration register >*/
	__vol uint32_t RTC_TAFCR;                                           /*< RTC tamper and alternate function configuration register >*/
	__vol uint32_t RTC_ALRMASSR;                                        /*< RTC alarm A sub second register >*/
	__vol uint32_t RTC_ALRMBSSR;                                        /*< RTC alarm B sub second register >*/
	__vol uint32_t RTC_BKPxR[20];                                       /*< RTC backup registers >*/

}RTC_Configuration_t;

#define DISC_RTC														 ((RTC_Configuration_t*) DISC_RTC_BASEADDR)   /*< RTC CONFIG MACRO >*/

typedef struct{
	__vol uint32_t WWDG_CR;                                              /*< WWDG Control register >*/
	__vol uint32_t WWDG_CFR;                                             /*< WWDG Configuration register >*/
	__vol uint32_t WWDG_SR;                                              /*< WWDG Status register >*/
}WWDG_Configuration_t;

#define DISC_WWDG														 ((WWDG_Configuration_t*) DISC_WWDG_BASEADDR)   /*< WWDG CONFIG MACRO >*/

typedef struct{
	__vol uint32_t IWDG_KR;                                              /*< IWDG Key register >*/
	__vol uint32_t IWDG_PR;                                              /*< IWDG Prescaler register >*/
	__vol uint32_t IWDG_RLR;                                             /*< IWDG Reload register >*/
	__vol uint32_t IWDG_SR;                                              /*< IWDG Status register >*/
}IWDG_Configuration_t;

#define DISC_IWDG														 ((IWDG_Configuration_t*) DISC_IWDG_BASEADDR)   /*< IWDG CONFIG MACRO >*/

typedef struct{
	__vol uint32_t SPI_CR1;                                              /*< SPI control register 1  >*/
	__vol uint32_t SPI_CR2;                                              /*< SPI control register 2 >*/
	__vol uint32_t SPI_SR;                                               /*< SPI status register >*/
	__vol uint32_t SPI_DR;                                               /*< SPI data register >*/
	__vol uint32_t SPI_CRCPR;                                            /*< SPI CRC polynomial register >*/
	__vol uint32_t SPI_RXCRCR;                                           /*< SPI RX CRC register >*/
	__vol uint32_t SPI_TXCRCR;                                           /*< SPI TX CRC register >*/
	__vol uint32_t SPI_I2SCFGR;                                          /*< SPI_I2S configuration register >*/
	__vol uint32_t SPI_I2SPR;                                            /*< SPI_I2S prescaler register >*/
}SPI_Configuration_t;

#define DISC_I2S2EXT														 ((SPI_Configuration_t*) DISC_I2S2EXT_BASEADDR)    /*< I2S2EXT CONFIG MACRO >*/
#define DISC_SPI2I2S2														 ((SPI_Configuration_t*) DISC_SPI2I2S2_BASEADDR)   /*< SPI2/I2S2 CONFIG MACRO >*/
#define DISC_SPI3I2S3														 ((SPI_Configuration_t*) DISC_SPI3I2S3_BASEADDR)   /*< SPI3/I2S3 CONFIG MACRO >*/
#define DISC_I2S3EXT														 ((SPI_Configuration_t*) DISC_I2S3EXT_BASEADDR)    /*< I2S3EXT CONFIG MACRO >*/
#define DISC_SPI1														 	 ((SPI_Configuration_t*) DISC_SPI1_BASEADDR)       /*< SPI1 CONFIG MACRO >*/
#define DISC_SPI4														 	 ((SPI_Configuration_t*) DISC_SPI4_BASEADDR)       /*< SPI4 CONFIG MACRO >*/
#define DISC_SPI5														 	 ((SPI_Configuration_t*) DISC_SPI5_BASEADDR)       /*< SPI5 CONFIG MACRO >*/
#define DISC_SPI6														 	 ((SPI_Configuration_t*) DISC_SPI6_BASEADDR)       /*< SPI6 CONFIG MACRO >*/

typedef struct{
	__vol uint32_t USART_SR;                                              /*< USART Status register  >*/
	__vol uint32_t USART_DR;                                              /*< USART Data register >*/
	__vol uint32_t USART_BRR;                                             /*< USART Baud rate register >*/
	__vol uint32_t USART_CR1;                                             /*< USART Control register 1 >*/
	__vol uint32_t USART_CR2;                                             /*< USART Control register 2 >*/
	__vol uint32_t USART_CR3;                                             /*< USART Control register 3 >*/
	__vol uint32_t USART_GTPR;                                            /*< USART Guard time and prescaler register >*/
}USART_Configuration_t;

#define DISC_USART1														 ((USART_Configuration_t*) DISC_USART1_BASEADDR)   /*< USART1 CONFIG MACRO >*/
#define DISC_USART6														 ((USART_Configuration_t*) DISC_USART6_BASEADDR)   /*< USART6 CONFIG MACRO >*/
#define DISC_USART2														 ((USART_Configuration_t*) DISC_USART2_BASEADDR)   /*< USART2 CONFIG MACRO >*/
#define DISC_USART3														 ((USART_Configuration_t*) DISC_USART3_BASEADDR)   /*< USART3 CONFIG MACRO >*/
#define DISC_UART4														 ((USART_Configuration_t*) DISC_UART4_BASEADDR)    /*< UART4 CONFIG MACRO >*/
#define DISC_UART5														 ((USART_Configuration_t*) DISC_UART5_BASEADDR)    /*< UART5 CONFIG MACRO >*/
#define DISC_UART7														 ((USART_Configuration_t*) DISC_UART7_BASEADDR)    /*< UART7 CONFIG MACRO >*/
#define DISC_UART8														 ((USART_Configuration_t*) DISC_UART8_BASEADDR)    /*< UART8 CONFIG MACRO >*/

typedef struct{
	__vol uint32_t I2C_CR1;                                              /*< I2C Control register 1  >*/
	__vol uint32_t I2C_CR2;                                              /*< I2C Control register 2 >*/
	__vol uint32_t I2C_OAR1;                                             /*< I2C Own address register 1  >*/
	__vol uint32_t I2C_OAR2;                                             /*< I2C Own address register 2 >*/
	__vol uint32_t I2C_DR;                                               /*< I2C Data register >*/
	__vol uint32_t I2C_SR1;                                              /*< I2C Status register 1 >*/
	__vol uint32_t I2C_SR2;                                              /*< I2C Status register 2 >*/
	__vol uint32_t I2C_CCR;                                              /*< I2C Clock control register >*/
	__vol uint32_t I2C_TRISE;                                            /*< I2C TRISE register >*/
	__vol uint32_t I2C_FLTR;                                             /*< I2C FLTR register >*/
}I2C_Configuration_t;

#define DISC_I2C1														 ((I2C_Configuration_t*) DISC_I2C1_BASEADDR)   /*< I2C1 CONFIG MACRO >*/
#define DISC_I2C2														 ((I2C_Configuration_t*) DISC_I2C2_BASEADDR)   /*< I2C2 CONFIG MACRO >*/
#define DISC_I2C3														 ((I2C_Configuration_t*) DISC_I2C3_BASEADDR)   /*< I2C3 CONFIG MACRO >*/

typedef struct{
	__vol uint32_t PWR_CR;                                              /*< PWR power control register >*/
	__vol uint32_t PWR_CSR;                                             /*< PWR power control/status register >*/
}PWR_Configuration_t;

#define DISC_PWR													 ((PWR_Configuration_t*) DISC_PWR_BASEADDR)   /*< PWR CONFIG MACRO >*/

typedef struct{
	__vol uint32_t DAC_CR;                                              /*< DAC control register  >*/
	__vol uint32_t DAC_SWTRIGR;                                         /*< DAC software trigger register >*/
	__vol uint32_t DAC_DHR12R1;                                         /*< DAC channel1 12-bit right-aligned data holding register >*/
	__vol uint32_t DAC_DHR12L1;                                         /*< DAC channel1 12-bit left aligned data holding register >*/
	__vol uint32_t DAC_DHR8R1;                                          /*< DAC channel1 8-bit right aligned data holding register >*/
	__vol uint32_t DAC_DHR12R2;                                         /*< DAC channel2 12-bit right aligned data holding register >*/
	__vol uint32_t DAC_DHR12L2;                                         /*< DAC channel2 12-bit left aligned data holding register  >*/
	__vol uint32_t DAC_DHR8R2;                                          /*< DAC channel2 8-bit right-aligned data holding register >*/
	__vol uint32_t DAC_DHR12RD;                                         /*< Dual DAC 12-bit right-aligned data holding register >*/
	__vol uint32_t DAC_DHR12LD;                                         /*< DUAL DAC 12-bit left aligned data holding register >*/
	__vol uint32_t DAC_DHR8RD;                                          /*< DUAL DAC 8-bit right aligned data holding register >*/
	__vol uint32_t DAC_DOR1;                                            /*< DAC channel1 data output register >*/
	__vol uint32_t DAC_DOR2;                                            /*< DAC channel2 data output register >*/
	__vol uint32_t DAC_SR;                                              /*< DAC status register >*/
}DAC_Configuration_t;

#define DISC_DAC														 ((DAC_Configuration_t*) DISC_DAC_BASEADDR)   /*< DAC CONFIG MACRO >*/

typedef struct{
	__vol uint32_t TIMx_CR1;                                              /*< TIMx control register >*/
	__vol uint32_t TIMx_CR2;                                              /*< TIMx control register 2 >*/
	__vol uint32_t TIMx_SMCR;                                             /*< TIMx slave mode control register >*/
	__vol uint32_t TIMx_DIER;                                             /*< TIMx DMA/Interrupt enable register >*/
	__vol uint32_t TIMx_SR;                                               /*< TIMx status register >*/
	__vol uint32_t TIMx_EGR;                                              /*< TIMx event generation register >*/
	__vol uint32_t TIMx_CCMR1;                                            /*< TIMx capture/compare mode register 1 >*/
	__vol uint32_t TIMx_CCMR2;                                            /*< TIMx capture/compare mode register 2 >*/
	__vol uint32_t TIMx_CCER;                                             /*< TIMx capture/compare enable register >*/
	__vol uint32_t TIMx_CNT;                                              /*< TIMx counter >*/
	__vol uint32_t TIMx_PSC;                                              /*< TIMx prescaler >*/
	__vol uint32_t TIMx_ARR;                                              /*< TIMx auto-reload register >*/
	__vol uint32_t TIMx_RCR;                                              /*< TIM1 and TIM8 repetition counter register >*/
	__vol uint32_t TIMx_CCR1;                                             /*< TIMx capture/compare register 1 >*/
	__vol uint32_t TIMx_CCR2;                                             /*< TIMx capture/compare register 2 >*/
	__vol uint32_t TIMx_CCR3;                                             /*< TIMx capture/compare register 3 >*/
	__vol uint32_t TIMx_CCR4;                                             /*< TIMx capture/compare register 4 >*/
	__vol uint32_t TIMx_BDTR;                                             /*< TIM1 and TIM8 break and dead-time register >*/
	__vol uint32_t TIMx_DCR;                                              /*< TIM1 and TIM8 DMA control register >*/
	__vol uint32_t TIMx_DMAR;                                             /*< TIM1 and TIM8 DMA address for full transfer >*/

}TIM18_Configuration_t;

#define DISC_TIM1														 ((TIM18_Configuration_t*) DISC_TIM1_BASEADDR)   /*< TIM1 CONFIG MACRO >*/
#define DISC_TIM8														 ((TIM18_Configuration_t*) DISC_TIM8_BASEADDR)   /*< TIM8 CONFIG MACRO >*/

typedef struct{
	__vol uint32_t ADC_SR;                                               /*< ADC status register >*/
	__vol uint32_t ADC_CR1;                                              /*< ADC control register 1 >*/
	__vol uint32_t ADC_CR2;                                              /*< ADC control register 2 >*/
	__vol uint32_t ADC_SMPR1;                                            /*< ADC sample time register 1 >*/
	__vol uint32_t ADC_SMPR2;                                            /*< ADC sample time register 2 >*/
	__vol uint32_t ADC_JOFR1;                                            /*< ADC injected channel data offset register 1 >*/
	__vol uint32_t ADC_JOFR2;                                            /*< ADC injected channel data offset register 2 >*/
	__vol uint32_t ADC_JOFR3;                                            /*< ADC injected channel data offset register 3 >*/
	__vol uint32_t ADC_JOFR4;                                            /*< ADC injected channel data offset register 4 >*/
	__vol uint32_t ADC_HTR;                                              /*< ADC watchdog higher threshold register >*/
	__vol uint32_t ADC_LTR;                                              /*< ADC watchdog lower threshold register >*/
	__vol uint32_t ADC_SQR1;                                             /*< ADC regular sequence register 1 >*/
	__vol uint32_t ADC_SQR2;                                             /*< ADC regular sequence register 2  >*/
	__vol uint32_t ADC_SQR3;                                             /*< ADC regular sequence register 3  >*/
	__vol uint32_t ADC_JSQR;                                             /*< ADC injected sequence register >*/
	__vol uint32_t ADC_JDR1;                                             /*< ADC injected data register 1 >*/
	__vol uint32_t ADC_JDR2;                                             /*< ADC injected data register 2 >*/
	__vol uint32_t ADC_JDR3;                                             /*< ADC injected data register 3 >*/
	__vol uint32_t ADC_JDR4;                                             /*< ADC injected data register 4 >*/
	__vol uint32_t ADC_DR;                                               /*< ADC regular data register >*/
	__vol uint32_t ADC_CSR;                                              /*< ADC Common status register >*/
	__vol uint32_t ADC_CCR;                                              /*< ADC common control register >*/
	__vol uint32_t ADC_CDR;                                              /*< ADC common regular data register for dual and triple modes >*/

}ADC_Configuration_t;

#define DISC_ADC1														 ((ADC_Configuration_t*) DISC_ADC1_BASEADDR)   /*< ADC1 CONFIG MACRO >*/
#define DISC_ADC2														 ((ADC_Configuration_t*) DISC_ADC2_BASEADDR)   /*< ADC2 CONFIG MACRO >*/
#define DISC_ADC3														 ((ADC_Configuration_t*) DISC_ADC3_BASEADDR)   /*< ADC3 CONFIG MACRO >*/


typedef struct{
	__vol uint32_t SDIO_POWER;                                            /*< SDIO power control register >*/
	__vol uint32_t SDIO_CLKCR;                                            /*< SDI clock control register >*/
	__vol uint32_t SDIO_ARG;                                              /*< SDIO argument register >*/
	__vol uint32_t SDIO_CMD;                                              /*< SDIO command register >*/
	__vol uint32_t SDIO_RESPCMD;                                          /*< SDIO command response register >*/
	__vol uint32_t SDIO_RESP1;                                            /*< SDIO response register 1 >*/
	__vol uint32_t SDIO_RESP2;                                            /*< SDIO response register 2 >*/
	__vol uint32_t SDIO_RESP3;                                            /*< SDIO response register 3 >*/
	__vol uint32_t SDIO_RESP4;                                            /*< SDIO response register 4 >*/
	__vol uint32_t SDIO_DTIMER;                                           /*< SDIO data timer register >*/
	__vol uint32_t SDIO_DLEN;                                             /*< SDIO data length register >*/
	__vol uint32_t SDIO_DCTRL;                                            /*< SDIO data control register >*/
	__vol uint32_t SDIO_DCOUNT;                                           /*< SDIO data counter register  >*/
	__vol uint32_t SDIO_STA;                                              /*< SDIO status register  >*/
	__vol uint32_t SDIO_ICR;                                              /*< SDIO interrupt clear register >*/
	__vol uint32_t SDIO_MASK;                                             /*< SDIO mask register >*/
	__vol uint32_t SDIO_FIFOCNT;                                          /*< SDIO FIFO counter register >*/
	__vol uint32_t SDIO_FIFO;                                             /*< SDIO data FIFO register >*/

}SDIO_Configuration_t;

#define DISC_SDIO														 ((SDIO_Configuration_t*) DISC_SDIO_BASEADDR)   /*< SDIO CONFIG MACRO >*/


typedef struct{
	__vol uint32_t SYSCFG_MEMRMP;                                          /*< SYSCFG memory remap register >*/
	__vol uint32_t SYSCFG_PMC;                                             /*< SYSCFG peripheral mode configuration register >*/
	__vol uint32_t SYSCFG_EXTICR[4];                                       /*< SYSCFG external interrupt configuration register 1-4 >*/
		  uint32_t RESERVED1[2];
	__vol uint32_t SYSCFG_CMPCR;                                           /*< Compensation cell control register >*/
		  uint32_t RESERVED2[2];
	__vol uint32_t SYSCFG_CFGR;                                           /*<  >*/
}SYSCFG_Configuration_t;

#define DISC_SYSCFG														 ((SYSCFG_Configuration_t*) DISC_SYSCFG_BASEADDR)   /*< SYSCFG CONFIG MACRO >*/


typedef struct{
	__vol uint32_t EXTI_IMR;                                            /*< EXTI Interrupt mask register >*/
	__vol uint32_t EXTI_EMR;                                            /*< EXTI Event mask register >*/
	__vol uint32_t EXTI_RTSR;                                           /*< EXTI Rising trigger selection register >*/
	__vol uint32_t EXTI_FTSR;                                           /*< EXTI Falling trigger selection register >*/
	__vol uint32_t EXTI_SWIER;                                          /*< EXTI Software interrupt event register >*/
	__vol uint32_t EXTI_PR;                                             /*< EXTI Pending register >*/


}EXTI_Configuration_t;

#define DISC_EXTI														 ((EXTI_Configuration_t*) DISC_EXTI_BASEADDR)   /*< EXTI CONFIG MACRO >*/


typedef struct{
	__vol uint32_t SAI_xCR1;                                            /*< SAI xConfiguration register 1  >*/
	__vol uint32_t SAI_xCR2;                                            /*< SAI xConfiguration register 2  >*/
	__vol uint32_t SAI_xFRCR;                                           /*< SAI xFrame configuration register >*/
	__vol uint32_t SAI_xSLOTR;                                          /*< SAI xSlot register >*/
	__vol uint32_t SAI_xIM;                                             /*< SAI xInterrupt mask register >*/
	__vol uint32_t SAI_xSR;                                             /*< SAI xStatus register >*/
	__vol uint32_t SAI_xCLRFR;                                          /*< SAI xClear flag register >*/
	__vol uint32_t SAI_xDR;                                             /*< SAI xData register >*/
}SAI_Configuration_t;

#define DISC_SAI														 ((SAI_Configuration_t*) DISC_SAI_BASEADDR)   /*< SAI CONFIG MACRO >*/


typedef struct{
	__vol uint32_t LTDC_SSCR;                                            /*< LTDC Synchronization Size Configuration Register >*/
	__vol uint32_t LTDC_BPCR;                                            /*< LTDC Back Porch Configuration Register >*/
	__vol uint32_t LTDC_AWCR;                                            /*< LTDC Active Width Configuration Register >*/
	__vol uint32_t LTDC_TWCR;                                            /*< LTDC Total Width Configuration Register >*/
	__vol uint32_t LTDC_GCR;                                             /*< LTDC Global Control Register >*/
	__vol uint32_t LTDC_SRCR;                                            /*< LTDC Shadow Reload Configuration Register >*/
	__vol uint32_t LTDC_BCCR;                                            /*< LTDC Background Color Configuration Register >*/
	__vol uint32_t LTDC_IER;                                             /*< LTDC Interrupt Enable Register >*/
	__vol uint32_t LTDC_ISR;                                             /*< LTDC Interrupt Status Register >*/
	__vol uint32_t LTDC_ICR;                                             /*< LTDC Interrupt Clear Register >*/
	__vol uint32_t LTDC_LIPCR;                                           /*< LTDC Line Interrupt Position Configuration Register >*/
	__vol uint32_t LTDC_CPSR;                                            /*< LTDC Current Position Status Register >*/
	__vol uint32_t LTDC_CDSR;                                            /*< LTDC Current Display Status Register  >*/
	__vol uint32_t LTDC_L1CR;                                            /*< LTDC Layer1 Control Register  >*/
	__vol uint32_t LTDC_L1WHPCR;                                         /*< LTDC Layer1 Window Horizontal Position Configuration Register >*/
	__vol uint32_t LTDC_L1WVPCR;                                         /*< LTDC Layer1 Window Vertical Position Configuration Register >*/
	__vol uint32_t LTDC_L1CKCR;                                          /*< LTDC Layer1 Color Keying Configuration Register >*/
	__vol uint32_t LTDC_L1PFCR;                                          /*< LTDC Layer1 Pixel Format Configuration Register >*/
	__vol uint32_t LTDC_L1CACR;                                          /*< LTDC Layer1 Constant Alpha Configuration Register >*/
	__vol uint32_t LTDC_L1DCCR;                                          /*< LTDC Layer1 Default Color Configuration Register >*/
	__vol uint32_t LTDC_L1BFCR;                                          /*< LTDC Layer1 Blending Factors Configuration Register >*/
	__vol uint32_t LTDC_L1CFBAR;                                         /*< LTDC Layer1 Color Frame Buffer Address Register  >*/
	__vol uint32_t LTDC_L1CFBLR;                                         /*< LTDC Layer1 Color Frame Buffer Length Register >*/
	__vol uint32_t LTDC_L1CFBLNR;                                        /*< LTDC Layer1 ColorFrame Buffer Line Number Register >*/
	__vol uint32_t LTDC_L1CLUTWR;                                        /*< LTDC Layer1 CLUT Write Register >*/
	__vol uint32_t LTDC_L2CR;                                            /*< LTDC Layer2 Control Register >*/
	__vol uint32_t LTDC_L2WHPCR;                                         /*< LTDC Layer2 Window Horizontal Position Configuration Register >*/
	__vol uint32_t LTDC_L2WVPCR;                                         /*< LTDC Layer2 Window Vertical Position Configuration Register >*/
	__vol uint32_t LTDC_L2CKCR;                                          /*< LTDC Layer2 Color Keying Configuration Register >*/
	__vol uint32_t LTDC_L2PFCR;                                          /*< LTDC Layer2 Pixel Format Configuration Register >*/
	__vol uint32_t LTDC_L2CACR;                                          /*< LTDC Layer2 Constant Alpha Configuration Register >*/
	__vol uint32_t LTDC_L2DCCR;                                          /*< LTDC Layer2 Default Color Configuration Register >*/
	__vol uint32_t LTDC_L2BFCR;                                          /*< LTDC Layer2 Blending Factors Configuration Register >*/
	__vol uint32_t LTDC_L2CFBAR;                                         /*< LTDC Layer2 Color Frame Buffer Address Register  >*/
	__vol uint32_t LTDC_L2CFBLR;                                         /*< LTDC Layer2 Color Frame Buffer Length Register >*/
	__vol uint32_t LTDC_L2CFBLNR;                                        /*< LTDC Layer2 ColorFrame Buffer Line Number Register >*/
	__vol uint32_t LTDC_L2CLUTWR;                                        /*< LTDC Layer2 CLUT Write Register >*/


}LTDC_Configuration_t;

#define DISC_LCDTFT														 ((LTDC_Configuration_t*) DISC_LCDTFT_BASEADDR)   /*< LCDTFT CONFIG MACRO >*/


/*
 * Clock Enable
 */

#define DISC_GPIOA_PCLK_EN() 											  (DISC_RCC->RCC_AHB1ENR |= (1 << 0))                 /*< ENABLE CLOCK GPIOA MACRO >*/
#define DISC_GPIOB_PCLK_EN() 											  (DISC_RCC->RCC_AHB1ENR |= (1 << 1))                 /*< ENABLE CLOCK GPIOB MACRO >*/
#define DISC_GPIOC_PCLK_EN() 											  (DISC_RCC->RCC_AHB1ENR |= (1 << 2))                 /*< ENABLE CLOCK GPIOC MACRO >*/
#define DISC_GPIOD_PCLK_EN() 											  (DISC_RCC->RCC_AHB1ENR |= (1 << 3))                 /*< ENABLE CLOCK GPIOD MACRO >*/
#define DISC_GPIOE_PCLK_EN() 											  (DISC_RCC->RCC_AHB1ENR |= (1 << 4))                 /*< ENABLE CLOCK GPIOE MACRO >*/
#define DISC_GPIOF_PCLK_EN() 											  (DISC_RCC->RCC_AHB1ENR |= (1 << 5))                 /*< ENABLE CLOCK GPIOF MACRO >*/
#define DISC_GPIOG_PCLK_EN() 											  (DISC_RCC->RCC_AHB1ENR |= (1 << 6))                 /*< ENABLE CLOCK GPIOG MACRO >*/
#define DISC_GPIOH_PCLK_EN() 											  (DISC_RCC->RCC_AHB1ENR |= (1 << 7))                 /*< ENABLE CLOCK GPIOH MACRO >*/
#define DISC_GPIOI_PCLK_EN() 											  (DISC_RCC->RCC_AHB1ENR |= (1 << 8))                 /*< ENABLE CLOCK GPIOI MACRO >*/
#define DISC_GPIOJ_PCLK_EN() 											  (DISC_RCC->RCC_AHB1ENR |= (1 << 9))                 /*< ENABLE CLOCK GPIOJ MACRO >*/
#define DISC_GPIOK_PCLK_EN() 											  (DISC_RCC->RCC_AHB1ENR |= (1 << 10))                /*< ENABLE CLOCK GPIOK MACRO >*/
#define DISC_CRC_PCLK_EN()												  (DISC_RCC->RCC_AHB1ENR |= (1 << 12))			      /*< ENABLE CLOCK CRC MACRO >*/
#define DISC_DMA1_PCLK_EN()												  (DISC_RCC->RCC_AHB1ENR |= (1 << 21))                /*< ENABLE CLOCK DMA1 MACRO >*/
#define DISC_DMA2_PCLK_EN()												  (DISC_RCC->RCC_AHB1ENR |= (1 << 22))                /*< ENABLE CLOCK DMA2 MACRO >*/
#define DISC_ETHERNETMAC_PCLK_EN()										  (DISC_RCC->RCC_AHB1ENR |= (1 << 25))                /*< ENABLE CLOCK ETHERNET MAC MACRO >*/
#define DISC_ETHERNETMACTX_PCLK_EN()									  (DISC_RCC->RCC_AHB1ENR |= (1 << 26))                /*< ENABLE CLOCK ETHERNET MAC TX MACRO >*/
#define DISC_ETHERNETMACRX_PCLK_EN()									  (DISC_RCC->RCC_AHB1ENR |= (1 << 27))                /*< ENABLE CLOCK ETHERNET MAC RX MACRO >*/
#define DISC_ETHERNETMACPTP_PCLK_EN()									  (DISC_RCC->RCC_AHB1ENR |= (1 << 28))                /*< ENABLE CLOCK ETHERNET MAC PTP MACRO >*/

#define DISC_DCMI_PCLK_EN()												  (DISC_RCC->RCC_AHB2ENR |= (1 << 0))			      /*< ENABLE CLOCK DCMI MACRO >*/
#define DISC_RNG_PCLK_EN()												  (DISC_RCC->RCC_AHB2ENR |= (1 << 4))                 /*< ENABLE CLOCK RNG MACRO >*/

#define DISC_TIM2_PCLK_EN() 											  (DISC_RCC->RCC_APB1ENR |= (1 << 0))                 /*< ENABLE CLOCK TIM2 MACRO >*/
#define DISC_TIM3_PCLK_EN() 											  (DISC_RCC->RCC_APB1ENR |= (1 << 1))                 /*< ENABLE CLOCK TIM3 MACRO >*/
#define DISC_TIM4_PCLK_EN() 											  (DISC_RCC->RCC_APB1ENR |= (1 << 2))                 /*< ENABLE CLOCK TIM4 MACRO >*/
#define DISC_TIM5_PCLK_EN() 											  (DISC_RCC->RCC_APB1ENR |= (1 << 3))                 /*< ENABLE CLOCK TIM5 MACRO >*/
#define DISC_TIM6_PCLK_EN() 											  (DISC_RCC->RCC_APB1ENR |= (1 << 4))                 /*< ENABLE CLOCK TIM6 MACRO >*/
#define DISC_TIM7_PCLK_EN() 											  (DISC_RCC->RCC_APB1ENR |= (1 << 5))                 /*< ENABLE CLOCK TIM7 MACRO >*/
#define DISC_TIM12_PCLK_EN() 											  (DISC_RCC->RCC_APB1ENR |= (1 << 6))                 /*< ENABLE CLOCK TIM12 MACRO >*/
#define DISC_TIM13_PCLK_EN() 											  (DISC_RCC->RCC_APB1ENR |= (1 << 7))                 /*< ENABLE CLOCK TIM13 MACRO >*/
#define DISC_TIM14_PCLK_EN() 											  (DISC_RCC->RCC_APB1ENR |= (1 << 8))                 /*< ENABLE CLOCK TIM14 MACRO >*/

#define DISC_WWDG_PCLK_EN() 											  (DISC_RCC->RCC_APB1ENR |= (1 << 11))                /*< ENABLE CLOCK WWDG MACRO >*/
#define DISC_SPI2_PCLK_EN() 											  (DISC_RCC->RCC_APB1ENR |= (1 << 14))                /*< ENABLE CLOCK SPI2 MACRO >*/
#define DISC_SPI3_PCLK_EN() 											  (DISC_RCC->RCC_APB1ENR |= (1 << 15))                /*< ENABLE CLOCK SPI3 MACRO >*/
#define DISC_USART2_PCLK_EN() 											  (DISC_RCC->RCC_APB1ENR |= (1 << 17))                /*< ENABLE CLOCK USART2 MACRO >*/
#define DISC_USART3_PCLK_EN() 											  (DISC_RCC->RCC_APB1ENR |= (1 << 18))                /*< ENABLE CLOCK USART3 MACRO >*/
#define DISC_UART4_PCLK_EN() 											  (DISC_RCC->RCC_APB1ENR |= (1 << 19))                /*< ENABLE CLOCK UART4 MACRO >*/
#define DISC_UART5_PCLK_EN() 											  (DISC_RCC->RCC_APB1ENR |= (1 << 20))                /*< ENABLE CLOCK UART5 MACRO >*/
#define DISC_I2C1_PCLK_EN() 											  (DISC_RCC->RCC_APB1ENR |= (1 << 21))                /*< ENABLE CLOCK I2C1 MACRO >*/
#define DISC_I2C2_PCLK_EN() 											  (DISC_RCC->RCC_APB1ENR |= (1 << 22))                /*< ENABLE CLOCK I2C2 MACRO >*/
#define DISC_I2C3_PCLK_EN() 											  (DISC_RCC->RCC_APB1ENR |= (1 << 23))                /*< ENABLE CLOCK I2C3 MACRO >*/
#define DISC_PWR_PCLK_EN() 											  	  (DISC_RCC->RCC_APB1ENR |= (1 << 28))                /*< ENABLE CLOCK PWR MACRO >*/
#define DISC_DAC_PCLK_EN() 											      (DISC_RCC->RCC_APB1ENR |= (1 << 29))                /*< ENABLE CLOCK DAC MACRO >*/

#define DISC_TIM1_PCLK_EN() 											  (DISC_RCC->RCC_APB2ENR |= (1 << 0))                /*< ENABLE CLOCK TIM1 MACRO >*/
#define DISC_TIM8_PCLK_EN() 											  (DISC_RCC->RCC_APB2ENR |= (1 << 1))                /*< ENABLE CLOCK TIM8 MACRO >*/
#define DISC_USART1_PCLK_EN() 											  (DISC_RCC->RCC_APB2ENR |= (1 << 4))                /*< ENABLE CLOCK USART1 MACRO >*/
#define DISC_USART6_PCLK_EN() 											  (DISC_RCC->RCC_APB2ENR |= (1 << 5))                /*< ENABLE CLOCK USART6 MACRO >*/
#define DISC_ADC1_PCLK_EN() 											  (DISC_RCC->RCC_APB2ENR |= (1 << 8))                /*< ENABLE CLOCK ADC1 MACRO >*/
#define DISC_ADC2_PCLK_EN() 											  (DISC_RCC->RCC_APB2ENR |= (1 << 9))                /*< ENABLE CLOCK ADC2 MACRO >*/
#define DISC_ADC3_PCLK_EN() 											  (DISC_RCC->RCC_APB2ENR |= (1 << 10))               /*< ENABLE CLOCK ADC3 MACRO >*/
#define DISC_SDIO_PCLK_EN() 											  (DISC_RCC->RCC_APB2ENR |= (1 << 11))               /*< ENABLE CLOCK SDIO MACRO >*/
#define DISC_SPI1_PCLK_EN() 											  (DISC_RCC->RCC_APB2ENR |= (1 << 12))               /*< ENABLE CLOCK SPI1 MACRO >*/
#define DISC_SPI4_PCLK_EN() 											  (DISC_RCC->RCC_APB2ENR |= (1 << 13))               /*< ENABLE CLOCK SPI4 MACRO >*/
#define DISC_SYSCFG_PCLK_EN() 											  (DISC_RCC->RCC_APB2ENR |= (1 << 14))               /*< ENABLE CLOCK SYSCFG MACRO >*/
#define DISC_TIM9_PCLK_EN() 											  (DISC_RCC->RCC_APB2ENR |= (1 << 16))               /*< ENABLE CLOCK TIM9 MACRO >*/
#define DISC_TIM10_PCLK_EN() 											  (DISC_RCC->RCC_APB2ENR |= (1 << 17))               /*< ENABLE CLOCK TIM10 MACRO >*/
#define DISC_TIM11_PCLK_EN() 											  (DISC_RCC->RCC_APB2ENR |= (1 << 18))               /*< ENABLE CLOCK TIM11 MACRO >*/
#define DISC_SPI5_PCLK_EN() 											  (DISC_RCC->RCC_APB2ENR |= (1 << 20))               /*< ENABLE CLOCK SPI5 MACRO >*/
#define DISC_SPI6_PCLK_EN() 											  (DISC_RCC->RCC_APB2ENR |= (1 << 21))               /*< ENABLE CLOCK SPI6 MACRO >*/
/*
 * Clock Disable
 */

#define DISC_GPIOA_PCLK_DIS() 											  (DISC_RCC->RCC_AHB1ENR &= ~(1 << 0))                 /*< DISABLE CLOCK GPIOA MACRO >*/
#define DISC_GPIOB_PCLK_DIS() 											  (DISC_RCC->RCC_AHB1ENR &= ~(1 << 1))                 /*< DISABLE CLOCK GPIOB MACRO >*/
#define DISC_GPIOC_PCLK_DIS() 											  (DISC_RCC->RCC_AHB1ENR &= ~(1 << 2))                 /*< DISABLE CLOCK GPIOC MACRO >*/
#define DISC_GPIOD_PCLK_DIS() 											  (DISC_RCC->RCC_AHB1ENR &= ~(1 << 3))                 /*< DISABLE CLOCK GPIOD MACRO >*/
#define DISC_GPIOE_PCLK_DIS() 											  (DISC_RCC->RCC_AHB1ENR &= ~(1 << 4))                 /*< DISABLE CLOCK GPIOE MACRO >*/
#define DISC_GPIOF_PCLK_DIS() 											  (DISC_RCC->RCC_AHB1ENR &= ~(1 << 5))                 /*< DISABLE CLOCK GPIOF MACRO >*/
#define DISC_GPIOG_PCLK_DIS() 											  (DISC_RCC->RCC_AHB1ENR &= ~(1 << 6))                 /*< DISABLE CLOCK GPIOG MACRO >*/
#define DISC_GPIOH_PCLK_DIS() 											  (DISC_RCC->RCC_AHB1ENR &= ~(1 << 7))                 /*< DISABLE CLOCK GPIOH MACRO >*/
#define DISC_GPIOI_PCLK_DIS() 											  (DISC_RCC->RCC_AHB1ENR &= ~(1 << 8))                 /*< DISABLE CLOCK GPIOI MACRO >*/
#define DISC_GPIOJ_PCLK_DIS() 											  (DISC_RCC->RCC_AHB1ENR &= ~(1 << 9))                 /*< DISABLE CLOCK GPIOJ MACRO >*/
#define DISC_GPIOK_PCLK_DIS() 											  (DISC_RCC->RCC_AHB1ENR &= ~(1 << 10))                 /*< DISABLE CLOCK GPIOJ MACRO >*/
#define DISC_CRC_PCLK_DIS()												  (DISC_RCC->RCC_AHB1ENR &= ~(1 << 12))			       /*< DISABLE CLOCK CRC MACRO >*/
#define DISC_DMA1_PCLK_DIS()											  (DISC_RCC->RCC_AHB1ENR &= ~(1 << 21))                /*< DISABLE CLOCK DMA1 MACRO >*/
#define DISC_DMA2_PCLK_DIS()											  (DISC_RCC->RCC_AHB1ENR &= ~(1 << 22))                /*< DISABLE CLOCK DMA2 MACRO >*/
#define DISC_ETHERNETMAC_PCLK_DIS()										  (DISC_RCC->RCC_AHB1ENR &= ~(1 << 25))                /*< DISABLE CLOCK ETHERNET MAC MACRO >*/
#define DISC_ETHERNETMACTX_PCLK_DIS()									  (DISC_RCC->RCC_AHB1ENR &= ~(1 << 26))                /*< DISABLE CLOCK ETHERNET MAC TX MACRO >*/
#define DISC_ETHERNETMACRX_PCLK_DIS()									  (DISC_RCC->RCC_AHB1ENR &= ~(1 << 27))                /*< DISABLE CLOCK ETHERNET MAC RX MACRO >*/
#define DISC_ETHERNETMACPTP_PCLK_DIS()									  (DISC_RCC->RCC_AHB1ENR &= ~(1 << 28))                /*< DISABLE CLOCK ETHERNET MAC PTP MACRO >*/

#define DISC_DCMI_PCLK_DIS()											  (DISC_RCC->RCC_AHB2ENR &= ~(1 << 0))			       /*< DISABLE CLOCK DCMI MACRO >*/
#define DISC_RNG_PCLK_DIS()												  (DISC_RCC->RCC_AHB2ENR &= ~(1 << 4))                 /*< DISABLE CLOCK RNG MACRO >*/

#define DISC_TIM2_PCLK_DIS() 											  (DISC_RCC->RCC_APB1ENR &= ~(1 << 0))                 /*< DISABLE CLOCK TIM2 MACRO >*/
#define DISC_TIM3_PCLK_DIS() 											  (DISC_RCC->RCC_APB1ENR &= ~(1 << 1))                 /*< DISABLE CLOCK TIM3 MACRO >*/
#define DISC_TIM4_PCLK_DIS() 											  (DISC_RCC->RCC_APB1ENR &= ~(1 << 2))                 /*< DISABLE CLOCK TIM4 MACRO >*/
#define DISC_TIM5_PCLK_DIS() 											  (DISC_RCC->RCC_APB1ENR &= ~(1 << 3))                 /*< DISABLE CLOCK TIM5 MACRO >*/
#define DISC_TIM6_PCLK_DIS() 											  (DISC_RCC->RCC_APB1ENR &= ~(1 << 4))                 /*< DISABLE CLOCK TIM6 MACRO >*/
#define DISC_TIM7_PCLK_DIS() 											  (DISC_RCC->RCC_APB1ENR &= ~(1 << 5))                 /*< DISABLE CLOCK TIM7 MACRO >*/
#define DISC_TIM12_PCLK_DIS() 											  (DISC_RCC->RCC_APB1ENR &= ~(1 << 6))                 /*< DISABLE CLOCK TIM12 MACRO >*/
#define DISC_TIM13_PCLK_DIS() 											  (DISC_RCC->RCC_APB1ENR &= ~(1 << 7))                 /*< DISABLE CLOCK TIM13 MACRO >*/
#define DISC_TIM14_PCLK_DIS() 											  (DISC_RCC->RCC_APB1ENR &= ~(1 << 8))                 /*< DISABLE CLOCK TIM14 MACRO >*/

#define DISC_WWDG_PCLK_DIS() 												  (DISC_RCC->RCC_APB1ENR &= ~(1 << 11))                /*< DISABLE CLOCK WWDG MACRO >*/
#define DISC_SPI2_PCLK_DIS() 											  (DISC_RCC->RCC_APB1ENR &= ~(1 << 14))                /*< DISABLE CLOCK SPI2 MACRO >*/
#define DISC_SPI3_PCLK_DIS() 											  (DISC_RCC->RCC_APB1ENR &= ~(1 << 15))                /*< DISABLE CLOCK SPI3 MACRO >*/
#define DISC_USART2_PCLK_DIS() 											  (DISC_RCC->RCC_APB1ENR &= ~(1 << 17))                /*< DISABLE CLOCK USART2 MACRO >*/
#define DISC_USART3_PCLK_DIS() 											  (DISC_RCC->RCC_APB1ENR &= ~(1 << 18))                /*< DISABLE CLOCK USART3 MACRO >*/
#define DISC_UART4_PCLK_DIS() 											  (DISC_RCC->RCC_APB1ENR &= ~(1 << 19))                /*< DISABLE CLOCK UART4 MACRO >*/
#define DISC_UART5_PCLK_DIS() 											  (DISC_RCC->RCC_APB1ENR &= ~(1 << 20))                /*< DISABLE CLOCK UART5 MACRO >*/
#define DISC_I2C1_PCLK_DIS() 											  (DISC_RCC->RCC_APB1ENR &= ~(1 << 21))                /*< DISABLE CLOCK I2C1 MACRO >*/
#define DISC_I2C2_PCLK_DIS() 											  (DISC_RCC->RCC_APB1ENR &= ~(1 << 22))                /*< DISABLE CLOCK I2C2 MACRO >*/
#define DISC_I2C3_PCLK_DIS() 											  (DISC_RCC->RCC_APB1ENR &= ~(1 << 23))                /*< DISABLE CLOCK I2C3 MACRO >*/
#define DISC_PWR_PCLK_DIS() 											  (DISC_RCC->RCC_APB1ENR &= ~(1 << 28))                /*< DISABLE CLOCK PWR MACRO >*/
#define DISC_DAC_PCLK_DIS() 											  (DISC_RCC->RCC_APB1ENR &= ~(1 << 29))                /*< DISABLE CLOCK DAC MACRO >*/

#define DISC_TIM1_PCLK_DIS() 											  (DISC_RCC->RCC_APB2ENR &= ~(1 << 0))                 /*< DISABLE CLOCK TIM1 MACRO >*/
#define DISC_TIM8_PCLK_DIS() 											  (DISC_RCC->RCC_APB2ENR &= ~(1 << 1))                 /*< DISABLE CLOCK TIM8 MACRO >*/
#define DISC_USART1_PCLK_DIS() 											  (DISC_RCC->RCC_APB2ENR &= ~(1 << 4))                 /*< DISABLE CLOCK USART1 MACRO >*/
#define DISC_USART6_PCLK_DIS() 											  (DISC_RCC->RCC_APB2ENR &= ~(1 << 5))                 /*< DISABLE CLOCK USART6 MACRO >*/
#define DISC_ADC1_PCLK_DIS() 											  (DISC_RCC->RCC_APB2ENR &= ~(1 << 8))                 /*< DISABLE CLOCK ADC1 MACRO >*/
#define DISC_ADC2_PCLK_DIS() 											  (DISC_RCC->RCC_APB2ENR &= ~(1 << 9))                 /*< DISABLE CLOCK ADC2 MACRO >*/
#define DISC_ADC3_PCLK_DIS() 											  (DISC_RCC->RCC_APB2ENR &= ~(1 << 10))                /*< DISABLE CLOCK ADC3 MACRO >*/
#define DISC_SDIO_PCLK_DIS() 											  (DISC_RCC->RCC_APB2ENR &= ~(1 << 11))                /*< DISABLE CLOCK SDIO MACRO >*/
#define DISC_SPI1_PCLK_DIS() 											  (DISC_RCC->RCC_APB2ENR &= ~(1 << 12))                /*< DISABLE CLOCK SPI1 MACRO >*/
#define DISC_SPI4_PCLK_DIS() 											  (DISC_RCC->RCC_APB2ENR &= ~(1 << 13))                /*< DISABLE CLOCK SPI4 MACRO >*/
#define DISC_SYSCFG_PCLK_DIS() 											  (DISC_RCC->RCC_APB2ENR &= ~(1 << 14))                /*< DISABLE CLOCK SYSCFG MACRO >*/
#define DISC_TIM9_PCLK_DIS() 											  (DISC_RCC->RCC_APB2ENR &= ~(1 << 16))                /*< DISABLE CLOCK TIM9 MACRO >*/
#define DISC_TIM10_PCLK_DIS() 											  (DISC_RCC->RCC_APB2ENR &= ~(1 << 17))                /*< DISABLE CLOCK TIM10 MACRO >*/
#define DISC_TIM11_PCLK_DIS() 											  (DISC_RCC->RCC_APB2ENR &= ~(1 << 18))                /*< DISABLE CLOCK TIM11 MACRO >*/
#define DISC_SPI5_PCLK_DIS() 											  (DISC_RCC->RCC_APB2ENR &= ~(1 << 20))                /*< DISABLE CLOCK SPI5 MACRO >*/
#define DISC_SPI6_PCLK_DIS() 											  (DISC_RCC->RCC_APB2ENR &= ~(1 << 21))                /*< DISABLE CLOCK SPI6 MACRO >*/
/*
 * Reset Register
 */

#define DISC_GPIOA_PCLK_RST() 											  do{(DISC_RCC->RCC_AHB1RSTR |= (1 << 0)); (DISC_RCC->RCC_AHB1RSTR &= ~(1 << 0));}while(0)                 /*< RESET CLOCK GPIOA MACRO >*/
#define DISC_GPIOB_PCLK_RST() 											  do{(DISC_RCC->RCC_AHB1RSTR |= (1 << 1)); (DISC_RCC->RCC_AHB1RSTR &= ~(1 << 1));}while(0)                 /*< RESET CLOCK GPIOB MACRO >*/
#define DISC_GPIOC_PCLK_RST() 											  do{(DISC_RCC->RCC_AHB1RSTR |= (1 << 2)); (DISC_RCC->RCC_AHB1RSTR &= ~(1 << 2));}while(0)                 /*< RESET CLOCK GPIOC MACRO >*/
#define DISC_GPIOD_PCLK_RST() 											  do{(DISC_RCC->RCC_AHB1RSTR |= (1 << 3)); (DISC_RCC->RCC_AHB1RSTR &= ~(1 << 3));}while(0)                 /*< RESET CLOCK GPIOD MACRO >*/
#define DISC_GPIOE_PCLK_RST() 											  do{(DISC_RCC->RCC_AHB1RSTR |= (1 << 4)); (DISC_RCC->RCC_AHB1RSTR &= ~(1 << 4));}while(0)                 /*< RESET CLOCK GPIOE MACRO >*/
#define DISC_GPIOF_PCLK_RST() 											  do{(DISC_RCC->RCC_AHB1RSTR |= (1 << 5)); (DISC_RCC->RCC_AHB1RSTR &= ~(1 << 5));}while(0)                 /*< RESET CLOCK GPIOF MACRO >*/
#define DISC_GPIOG_PCLK_RST() 											  do{(DISC_RCC->RCC_AHB1RSTR |= (1 << 6)); (DISC_RCC->RCC_AHB1RSTR &= ~(1 << 6));}while(0)                 /*< RESET CLOCK GPIOG MACRO >*/
#define DISC_GPIOH_PCLK_RST() 											  do{(DISC_RCC->RCC_AHB1RSTR |= (1 << 7)); (DISC_RCC->RCC_AHB1RSTR &= ~(1 << 7));}while(0)                 /*< RESET CLOCK GPIOH MACRO >*/
#define DISC_GPIOI_PCLK_RST() 											  do{(DISC_RCC->RCC_AHB1RSTR |= (1 << 8)); (DISC_RCC->RCC_AHB1RSTR &= ~(1 << 8));}while(0)                 /*< RESET CLOCK GPIOI MACRO >*/
#define DISC_GPIOJ_PCLK_RST() 											  do{(DISC_RCC->RCC_AHB1RSTR |= (1 << 9)); (DISC_RCC->RCC_AHB1RSTR &= ~(1 << 9));}while(0)                 /*< RESET CLOCK GPIOJ MACRO >*/
#define DISC_GPIOK_PCLK_RST() 											  do{(DISC_RCC->RCC_AHB1RSTR |= (1 << 10)); (DISC_RCC->RCC_AHB1RSTR &= ~(1 << 10));}while(0)                 /*< RESET CLOCK GPIOK MACRO >*/
#define DISC_CRC_PCLK_RST()												  do{(DISC_RCC->RCC_AHB1RSTR |= (1 << 12)); (DISC_RCC->RCC_AHB1RSTR &= ~(1 << 12));}while(0)			       /*< RESET CLOCK CRC MACRO >*/
#define DISC_DMA1_PCLK_RST()											  do{(DISC_RCC->RCC_AHB1RSTR |= (1 << 21)); (DISC_RCC->RCC_AHB1RSTR &= ~(1 << 21));}while(0)               /*< RESET CLOCK DMA1 MACRO >*/
#define DISC_DMA2_PCLK_RST()											  do{(DISC_RCC->RCC_AHB1RSTR |= (1 << 22)); (DISC_RCC->RCC_AHB1RSTR &= ~(1 << 22));}while(0)                /*< RESET CLOCK DMA2 MACRO >*/
#define DISC_ETHERNETMAC_PCLK_RST()										  do{(DISC_RCC->RCC_AHB1RSTR |= (1 << 25)); (DISC_RCC->RCC_AHB1RSTR &= ~(1 << 25));}while(0)                /*< RESET CLOCK ETHERNET MAC MACRO >*/
#define DISC_ETHERNETMACTX_PCLK_RST()									  do{(DISC_RCC->RCC_AHB1RSTR |= (1 << 26)); (DISC_RCC->RCC_AHB1RSTR &= ~(1 << 26));}while(0)                /*< RESET CLOCK ETHERNET MAC TX MACRO >*/
#define DISC_ETHERNETMACRX_PCLK_RST()									  do{(DISC_RCC->RCC_AHB1RSTR |= (1 << 27)); (DISC_RCC->RCC_AHB1RSTR &= ~(1 << 27));}while(0)                /*< RESET CLOCK ETHERNET MAC RX MACRO >*/
#define DISC_ETHERNETMACPTP_PCLK_RST()									  do{(DISC_RCC->RCC_AHB1RSTR |= (1 << 28)); (DISC_RCC->RCC_AHB1RSTR &= ~(1 << 28));}while(0)                /*< RESET CLOCK ETHERNET MAC PTP MACRO >*/

#define DISC_DCMI_PCLK_RST()											  do{(DISC_RCC->RCC_AHB2RSTR |= (1 << 0)); (DISC_RCC->RCC_AHB2RSTR &= ~(1 << 0));}while(0) 			       /*< RESET CLOCK DCMI MACRO >*/
#define DISC_RNG_PCLK_RST()												  do{(DISC_RCC->RCC_AHB2RSTR |= (1 << 4)); (DISC_RCC->RCC_AHB2RSTR &= ~(1 << 4));}while(0)                  /*< RESET CLOCK RNG MACRO >*/

#define DISC_TIM2_PCLK_RST() 											  do{(DISC_RCC->RCC_APB1RSTR |= (1 << 0)); (DISC_RCC->RCC_APB1RSTR &= ~(1 << 0));}while(0)                 /*< RESET CLOCK TIM2 MACRO >*/
#define DISC_TIM3_PCLK_RST() 											  do{(DISC_RCC->RCC_APB1RSTR |= (1 << 1)); (DISC_RCC->RCC_APB1RSTR &= ~(1 << 1));}while(0)                 /*< RESET CLOCK TIM3 MACRO >*/
#define DISC_TIM4_PCLK_RST() 											  do{(DISC_RCC->RCC_APB1RSTR |= (1 << 2)); (DISC_RCC->RCC_APB1RSTR &= ~(1 << 2));}while(0)                 /*< RESET CLOCK TIM4 MACRO >*/
#define DISC_TIM5_PCLK_RST() 											  do{(DISC_RCC->RCC_APB1RSTR |= (1 << 3)); (DISC_RCC->RCC_APB1RSTR &= ~(1 << 3));}while(0)                 /*< RESET CLOCK TIM5 MACRO >*/
#define DISC_TIM6_PCLK_RST() 											  do{(DISC_RCC->RCC_APB1RSTR |= (1 << 4)); (DISC_RCC->RCC_APB1RSTR &= ~(1 << 4));}while(0)                 /*< RESET CLOCK TIM6 MACRO >*/
#define DISC_TIM7_PCLK_RST() 											  do{(DISC_RCC->RCC_APB1RSTR |= (1 << 5)); (DISC_RCC->RCC_APB1RSTR &= ~(1 << 5));}while(0)                 /*< RESET CLOCK TIM7 MACRO >*/
#define DISC_TIM12_PCLK_RST() 											  do{(DISC_RCC->RCC_APB1RSTR |= (1 << 6)); (DISC_RCC->RCC_APB1RSTR &= ~(1 << 6));}while(0)                 /*< RESET CLOCK TIM12 MACRO >*/
#define DISC_TIM13_PCLK_RST() 											  do{(DISC_RCC->RCC_APB1RSTR |= (1 << 7)); (DISC_RCC->RCC_APB1RSTR &= ~(1 << 7));}while(0)                /*< RESET CLOCK TIM13 MACRO >*/
#define DISC_TIM14_PCLK_RST() 											  do{(DISC_RCC->RCC_APB1RSTR |= (1 << 8)); (DISC_RCC->RCC_APB1RSTR &= ~(1 << 8));}while(0)                 /*< RESET CLOCK TIM14 MACRO >*/

#define DISC_WWDG_PCLK_RST() 											  do{(DISC_RCC->RCC_APB1RSTR |= (1 << 11)); (DISC_RCC->RCC_APB1RSTR &= ~(1 << 11));}while(0)                /*< RESET CLOCK WWDG MACRO >*/
#define DISC_SPI2_PCLK_RST() 											  do{(DISC_RCC->RCC_APB1RSTR |= (1 << 14)); (DISC_RCC->RCC_APB1RSTR &= ~(1 << 14));}while(0)                /*< RESET CLOCK SPI2 MACRO >*/
#define DISC_SPI3_PCLK_RST() 											  do{(DISC_RCC->RCC_APB1RSTR |= (1 << 15)); (DISC_RCC->RCC_APB1RSTR &= ~(1 << 15));}while(0)                /*< RESET CLOCK SPI3 MACRO >*/
#define DISC_USART2_PCLK_RST() 											  do{(DISC_RCC->RCC_APB1RSTR |= (1 << 17)); (DISC_RCC->RCC_APB1RSTR &= ~(1 << 17));}while(0)                /*< RESET CLOCK USART2 MACRO >*/
#define DISC_USART3_PCLK_RST() 											  do{(DISC_RCC->RCC_APB1RSTR |= (1 << 18)); (DISC_RCC->RCC_APB1RSTR &= ~(1 << 18));}while(0)                /*< RESET CLOCK USART3 MACRO >*/
#define DISC_UART4_PCLK_RST() 											  do{(DISC_RCC->RCC_APB1RSTR |= (1 << 19)); (DISC_RCC->RCC_APB1RSTR &= ~(1 << 19));}while(0)                /*< RESET CLOCK UART4 MACRO >*/
#define DISC_UART5_PCLK_RST() 											  do{(DISC_RCC->RCC_APB1RSTR |= (1 << 20)); (DISC_RCC->RCC_APB1RSTR &= ~(1 << 20));}while(0)                /*< RESET CLOCK UART5 MACRO >*/
#define DISC_I2C1_PCLK_RST() 											  do{(DISC_RCC->RCC_APB1RSTR |= (1 << 21)); (DISC_RCC->RCC_APB1RSTR &= ~(1 << 21));}while(0)                /*< RESET CLOCK I2C1 MACRO >*/
#define DISC_I2C2_PCLK_RST() 											  do{(DISC_RCC->RCC_APB1RSTR |= (1 << 22)); (DISC_RCC->RCC_APB1RSTR &= ~(1 << 22));}while(0)                /*< RESET CLOCK I2C2 MACRO >*/
#define DISC_I2C3_PCLK_RST() 											  do{(DISC_RCC->RCC_APB1RSTR |= (1 << 23)); (DISC_RCC->RCC_APB1RSTR &= ~(1 << 23));}while(0)                /*< RESET CLOCK I2C3 MACRO >*/
#define DISC_PWR_PCLK_RST() 											  do{(DISC_RCC->RCC_APB1RSTR |= (1 << 28)); (DISC_RCC->RCC_APB1RSTR &= ~(1 << 28));}while(0)                /*< RESET CLOCK PWR MACRO >*/
#define DISC_DAC_PCLK_RST() 											  do{(DISC_RCC->RCC_APB1RSTR |= (1 << 29)); (DISC_RCC->RCC_APB1RSTR &= ~(1 << 29));}while(0)                /*< RESET CLOCK DAC MACRO >*/

#define DISC_TIM1_PCLK_RST() 											  do{(DISC_RCC->RCC_APB2RSTR |= (1 << 0)); (DISC_RCC->RCC_APB2RSTR &= ~(1 << 0));}while(0)                 /*< RESET CLOCK TIM1 MACRO >*/
#define DISC_TIM8_PCLK_RST() 											  do{(DISC_RCC->RCC_APB2RSTR |= (1 << 1)); (DISC_RCC->RCC_APB2RSTR &= ~(1 << 1));}while(0)                 /*< RESET CLOCK TIM8 MACRO >*/
#define DISC_USART1_PCLK_RST() 											  do{(DISC_RCC->RCC_APB2RSTR |= (1 << 4)); (DISC_RCC->RCC_APB2RSTR &= ~(1 << 4));}while(0)                 /*< RESET CLOCK USART1 MACRO >*/
#define DISC_USART6_PCLK_RST() 											  do{(DISC_RCC->RCC_APB2RSTR |= (1 << 5)); (DISC_RCC->RCC_APB2RSTR &= ~(1 << 5));}while(0)                 /*< RESET CLOCK USART6 MACRO >*/
#define DISC_ADC1_PCLK_RST() 											  do{(DISC_RCC->RCC_APB2RSTR |= (1 << 8)); (DISC_RCC->RCC_APB2RSTR &= ~(1 << 8));}while(0)                 /*< RESET CLOCK ADC1 MACRO >*/
#define DISC_ADC2_PCLK_RST() 											  do{(DISC_RCC->RCC_APB2RSTR |= (1 << 9)); (DISC_RCC->RCC_APB2RSTR &= ~(1 << 9));}while(0)                 /*< RESET CLOCK ADC2 MACRO >*/
#define DISC_ADC3_PCLK_RST() 											  do{(DISC_RCC->RCC_APB2RSTR |= (1 << 10)); (DISC_RCC->RCC_APB2RSTR &= ~(1 << 10));}while(0)                /*< RESET CLOCK ADC3 MACRO >*/
#define DISC_SDIO_PCLK_RST() 											  do{(DISC_RCC->RCC_APB2RSTR |= (1 << 11)); (DISC_RCC->RCC_APB2RSTR &= ~(1 << 11));}while(0)               /*< RESET CLOCK SDIO MACRO >*/
#define DISC_SPI1_PCLK_RST() 											  do{(DISC_RCC->RCC_APB2RSTR |= (1 << 12)); (DISC_RCC->RCC_APB2RSTR &= ~(1 << 12));}while(0)                /*< RESET CLOCK SPI1 MACRO >*/
#define DISC_SPI4_PCLK_RST() 											  do{(DISC_RCC->RCC_APB2RSTR |= (1 << 13)); (DISC_RCC->RCC_APB2RSTR &= ~(1 << 13));}while(0)
#define DISC_SYSCFG_PCLK_RST() 											  do{(DISC_RCC->RCC_APB2RSTR |= (1 << 14)); (DISC_RCC->RCC_APB2RSTR &= ~(1 << 14));}while(0)                /*< RESET CLOCK SYSCFG MACRO >*/
#define DISC_TIM9_PCLK_RST() 											  do{(DISC_RCC->RCC_APB2RSTR |= (1 << 16)); (DISC_RCC->RCC_APB2RSTR &= ~(1 << 16));}while(0)               /*< RESET CLOCK TIM9 MACRO >*/
#define DISC_TIM10_PCLK_RST() 											  do{(DISC_RCC->RCC_APB2RSTR |= (1 << 17)); (DISC_RCC->RCC_APB2RSTR &= ~(1 << 17));}while(0)                /*< RESET CLOCK TIM10 MACRO >*/
#define DISC_TIM11_PCLK_RST() 											  do{(DISC_RCC->RCC_APB2RSTR |= (1 << 18)); (DISC_RCC->RCC_APB2RSTR &= ~(1 << 18));}while(0)                /*< RESET CLOCK TIM11 MACRO >*/
#define DISC_SPI5_PCLK_RST() 											  do{(DISC_RCC->RCC_APB2RSTR |= (1 << 20)); (DISC_RCC->RCC_APB2RSTR &= ~(1 << 20));}while(0)
#define DISC_SPI6_PCLK_RST() 											  do{(DISC_RCC->RCC_APB2RSTR |= (1 << 21)); (DISC_RCC->RCC_APB2RSTR &= ~(1 << 21));}while(0)






#endif /* INC_STM32F407DISC1_H_ */






