#ifndef INC_STM32F407DISC1_I2C_DRIVER_H_
#define INC_STM32F407DISC1_I2C_DRIVER_H_

#include "stm32f407disc1.h"




typedef struct{

	uint32_t I2Cx_SCLSpeed; //@I2Cx_SCLSpeed
	uint8_t I2Cx_DeviceAddress;
	uint8_t I2Cx_ACKControl; //@I2Cx_ACKControl
	uint16_t I2Cx_FMDutyCycle; //@I2Cx_FMDutyCycle

}I2Cx_PinConfig_t;


typedef struct{

	I2C_Configuration_t *pI2Cx;
	I2Cx_PinConfig_t I2Cx_PinConfig;

}I2Cx_Handle_t;


/*
 * @I2Cx_SCLSpeed
 */
#define I2Cx_SCLSPEED_SM 		100000
#define I2Cx_SCLSPEED_FM2K 		200000
#define I2Cx_SCLSPEED_FM4K 		400000


/*
 * @I2Cx_ACKControl
 */
#define I2Cx_ACKCONTROL_EN 		1
#define I2Cx_ACKCONTROL_DEN 	0

/*
 * @I2Cx_FMDutyCycle
 */
#define I2Cx_FMDUTY_2			0
#define I2Cx_FMDUTY_16_9		1


/*
 * APIs
 */

// Clock
void I2C_PCLKControl(I2C_Configuration_t *pI2Cx, uint8_t status);

//Init
void I2C_Init(I2Cx_Handle_t *pI2Cx_Handle);
void I2C_DeInit(I2C_Configuration_t *pI2Cx);
void I2C_Status(I2C_Configuration_t *pI2Cx, uint8_t status);

//Master Send
void I2C_MasterSendData(I2Cx_Handle_t *pI2Cx_Handle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr);

//IRQ
void I2C_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t state);


//MISC
uint8_t I2C_GetFlagStatus(I2C_Configuration_t *pI2Cx, uint8_t flag);

void I2C_ApplicationCallback(I2Cx_Handle_t *pI2Cx_Handle, uint8_t event);




//CR1 Macros

#define I2Cx_REG_CR1_PE				0
#define I2Cx_REG_CR1_SMBUS			1
#define I2Cx_REG_CR1_SMBTYPE		3
#define I2Cx_REG_CR1_ENARP			4
#define I2Cx_REG_CR1_ENPEC			5
#define I2Cx_REG_CR1_ENGC			6
#define I2Cx_REG_CR1_NOSTRETCH		7
#define I2Cx_REG_CR1_START			8
#define I2Cx_REG_CR1_STOP			9
#define I2Cx_REG_CR1_ACK			10
#define I2Cx_REG_CR1_POS			11
#define I2Cx_REG_CR1_PEC			12
#define I2Cx_REG_CR1_ALERT			13
#define I2Cx_REG_CR1_SWRST			15

//CR2 Macros

#define I2Cx_REG_CR2_FREQ			0
#define I2Cx_REG_CR2_ITERREN		8
#define I2Cx_REG_CR2_ITEVTEN		9
#define I2Cx_REG_CR2_ITBUFEN		10
#define I2Cx_REG_CR2_DMAEN			11
#define I2Cx_REG_CR2_LAST			12

//SR1 Macros

#define I2Cx_REG_SR1_SB				0
#define I2Cx_REG_SR1_ADDR			1
#define I2Cx_REG_SR1_BTF			2
#define I2Cx_REG_SR1_ADD10			3
#define I2Cx_REG_SR1_STOPF			4
#define I2Cx_REG_SR1_RxNE			6
#define I2Cx_REG_SR1_TxE			7
#define I2Cx_REG_SR1_BERR			8
#define I2Cx_REG_SR1_ARLO			9
#define I2Cx_REG_SR1_AF				10
#define I2Cx_REG_SR1_OVR			11
#define I2Cx_REG_SR1_PECERR			12
#define I2Cx_REG_SR1_TIMEOUT		14
#define I2Cx_REG_SR1_SMBALERT		15

//SR2 Macros

#define I2Cx_REG_SR2_MSL			0
#define I2Cx_REG_SR2_BUSY			1
#define I2Cx_REG_SR2_TRA			2
#define I2Cx_REG_SR2_GENCALL		4
#define I2Cx_REG_SR2_SMBDEFAULT		5
#define I2Cx_REG_SR2_SMBHOST		6
#define I2Cx_REG_SR2_DUALF			7
#define I2Cx_REG_SR2_PEC			8


//CCR Macros

#define I2Cx_REG_CCR_CCR			0
#define I2Cx_REG_CCR_DUTY			14
#define I2Cx_REG_CCR_FS				15


//Flags

#define I2Cx_FLAG_SB						(1 << I2Cx_REG_SR1_SB)
#define I2Cx_FLAG_TXE						(1 << I2Cx_REG_SR1_TxE)
#define I2Cx_FLAG_RXNE						(1 << I2Cx_REG_SR1_RxNE)
#define I2Cx_FLAG_OVR						(1 << I2Cx_REG_SR1_OVR)
#define I2Cx_FLAG_AF						(1 << I2Cx_REG_SR1_AF)
#define I2Cx_FLAG_ARLO						(1 << I2Cx_REG_SR1_ARLO)
#define I2Cx_FLAG_BERR						(1 << I2Cx_REG_SR1_BERR)
#define I2Cx_FLAG_STOPF						(1 << I2Cx_REG_SR1_STOPF)
#define I2Cx_FLAG_ADD10						(1 << I2Cx_REG_SR1_ADD10)
#define I2Cx_FLAG_BTF						(1 << I2Cx_REG_SR1_BTF)
#define I2Cx_FLAG_ADDR						(1 << I2Cx_REG_SR1_ADDR)
#define I2Cx_FLAG_TIMEOUT					(1 << I2Cx_REG_SR1_TIMEOUT)




#endif /* INC_STM32F407DISC1_I2C_DRIVER_H_ */
