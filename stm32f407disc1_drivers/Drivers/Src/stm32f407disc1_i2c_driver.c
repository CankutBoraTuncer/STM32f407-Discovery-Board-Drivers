#include "stm32f407disc1_i2c_driver.h"


uint32_t RCC_GetPCLK1Value(void);
static void I2C_GenerateStartCondition(I2C_Configuration_t *pI2Cx);
static void I2C_ExecuteAddrPhase(I2C_Configuration_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Configuration_t *pI2Cx);
static void I2C_SendData(I2C_Configuration_t *pI2Cx, uint8_t *pTxBuffer, uint32_t Len);
static void I2C_GenerateStopCondition(I2C_Configuration_t *pI2Cx);

uint16_t AHB_PreScaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint8_t APB1_PreScaler[4] = {2, 4, 8, 16};


static void I2C_GenerateStartCondition(I2C_Configuration_t *pI2Cx){
	pI2Cx->I2C_CR1 |= (1 << I2Cx_REG_CR1_START);
}

static void I2C_ExecuteAddrPhase(I2C_Configuration_t *pI2Cx, uint8_t SlaveAddr){
	SlaveAddr = SlaveAddr << 1 ;
	SlaveAddr &= ~(1);
	pI2Cx->I2C_DR = SlaveAddr;
}

static void I2C_ClearADDRFlag(I2C_Configuration_t *pI2Cx){
	uint32_t tempreg;
	tempreg = pI2Cx->I2C_SR1;
	tempreg = pI2Cx->I2C_SR2;
	(void)tempreg;
}

static void I2C_SendData(I2C_Configuration_t *pI2Cx, uint8_t *pTxBuffer, uint32_t Len){
	while(Len != 0){
		while(! I2C_GetFlagStatus(pI2Cx, I2Cx_FLAG_TXE));
		pI2Cx->I2C_DR = *pTxBuffer;
		pTxBuffer ++;
		Len --;
	}
}

static void I2C_GenerateStopCondition(I2C_Configuration_t *pI2Cx){
	pI2Cx->I2C_CR1 |= (1 << I2Cx_REG_CR1_STOP);
}


// Clock
void I2C_PCLKControl(I2C_Configuration_t *pI2Cx, uint8_t status){

	if(status == ENABLE){
		if(pI2Cx == DISC_I2C1)	{
			DISC_I2C1_PCLK_EN();
		}
		else if(pI2Cx == DISC_I2C2)	{
			DISC_I2C2_PCLK_EN();
		}
		else if(pI2Cx == DISC_I2C3)	{
			DISC_I2C3_PCLK_EN();
		}
	}
	else{
		if(pI2Cx == DISC_I2C1)	{
			DISC_I2C1_PCLK_DIS();
		}
		else if(pI2Cx == DISC_I2C2)	{
			DISC_I2C2_PCLK_DIS();
		}
		else if(pI2Cx == DISC_I2C3)	{
			DISC_I2C3_PCLK_DIS();
		}
	}
}

void I2C_Init(I2Cx_Handle_t *pI2Cx_Handle){

	uint32_t tempreg = 0;

	//Enable ACKing
	tempreg |= (pI2Cx_Handle->I2Cx_PinConfig.I2Cx_ACKControl << 10);
	pI2Cx_Handle->pI2Cx->I2C_CR1 = tempreg;

	//Conf FREQ
	tempreg = 0;
	tempreg = RCC_GetPCLK1Value() / 1000000U;
	pI2Cx_Handle->pI2Cx->I2C_CR2 = tempreg & 0x3F;

	//Slave Addr
	tempreg = 0;
	tempreg |= pI2Cx_Handle->I2Cx_PinConfig.I2Cx_DeviceAddress << 1;
	tempreg |= (1 << 14);
	pI2Cx_Handle->pI2Cx->I2C_OAR1 = tempreg & 0x7F;

	//CCR Calc
	uint16_t ccr_val = 0;
	tempreg = 0;

	if(pI2Cx_Handle->I2Cx_PinConfig.I2Cx_SCLSpeed <= I2Cx_SCLSPEED_SM){
		ccr_val = (RCC_GetPCLK1Value() / (2 * pI2Cx_Handle->I2Cx_PinConfig.I2Cx_SCLSpeed));
		tempreg |= ccr_val & 0xFFF;

	}
	else{
		tempreg |= (1 << 15);
		tempreg |= (pI2Cx_Handle->I2Cx_PinConfig.I2Cx_FMDutyCycle << 14);
		if(pI2Cx_Handle->I2Cx_PinConfig.I2Cx_FMDutyCycle == I2Cx_FMDUTY_2){
			ccr_val = (RCC_GetPCLK1Value() / (3 * pI2Cx_Handle->I2Cx_PinConfig.I2Cx_SCLSpeed));
		}
		else if(pI2Cx_Handle->I2Cx_PinConfig.I2Cx_FMDutyCycle == I2Cx_FMDUTY_16_9){
			ccr_val = (RCC_GetPCLK1Value() / (25 * pI2Cx_Handle->I2Cx_PinConfig.I2Cx_SCLSpeed));
		}
		tempreg |= ccr_val & 0xFFF;
	}

	pI2Cx_Handle->pI2Cx->I2C_CCR = tempreg;

	//Slew Rate
	if(pI2Cx_Handle->I2Cx_PinConfig.I2Cx_SCLSpeed <= I2Cx_SCLSPEED_SM) tempreg = (RCC_GetPCLK1Value() / 1000000U) + 1 ;

	else tempreg = ((RCC_GetPCLK1Value() * 300) / 1000000000U) + 1 ;

	pI2Cx_Handle->pI2Cx->I2C_TRISE = tempreg & 0x3F;

}


void I2C_DeInit(I2C_Configuration_t *pI2Cx){

	if(pI2Cx == DISC_I2C1)	{
		DISC_I2C1_PCLK_RST();
	}
	else if(pI2Cx == DISC_I2C2)	{
		DISC_I2C2_PCLK_RST();
	}
	else if(pI2Cx == DISC_I2C3)	{
		DISC_I2C3_PCLK_RST();
	}

}

void I2C_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t state){

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



void I2C_MasterSendData(I2Cx_Handle_t *pI2Cx_Handle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr){

	I2C_GenerateStartCondition(pI2Cx_Handle->pI2Cx);

	while(! I2C_GetFlagStatus(pI2Cx_Handle->pI2Cx ,I2Cx_FLAG_SB));

	I2C_ExecuteAddrPhase(pI2Cx_Handle->pI2Cx, pI2Cx_Handle->I2Cx_PinConfig.I2Cx_DeviceAddress);

	while(! I2C_GetFlagStatus(pI2Cx_Handle->pI2Cx, I2Cx_FLAG_ADDR));

	I2C_ClearADDRFlag(pI2Cx_Handle->pI2Cx);

	I2C_SendData(pI2Cx_Handle->pI2Cx, pTxBuffer, Len);

	while(! I2C_GetFlagStatus(pI2Cx_Handle->pI2Cx, I2Cx_FLAG_TXE));

	while(! I2C_GetFlagStatus(pI2Cx_Handle->pI2Cx, I2Cx_FLAG_BTF));

	I2C_GenerateStopCondition(pI2Cx_Handle->pI2Cx);

}



uint8_t I2C_GetFlagStatus(I2C_Configuration_t *pI2Cx, uint8_t flag){
	return (pI2Cx->I2C_SR1 & flag);
}



void I2C_Status(I2C_Configuration_t *pI2Cx, uint8_t status){

	if(status == ENABLE){
		pI2Cx->I2C_CR1 |= (1 << I2Cx_REG_CR1_PE);
	}
	else if (status == DISABLE){
		pI2Cx->I2C_CR1 &= ~(1 << I2Cx_REG_CR1_PE);
	}

}

uint32_t RCC_GetPCLK1Value(void){

	uint32_t pclk1,sysclk;
	uint8_t clkSource,temp,ahbp, apb1;

	clkSource = ((DISC_RCC->RCC_CFGR >> 2) & 0x3);

	if(clkSource == 0){
		sysclk = 16000000;
	}
	else if(clkSource == 1){
		sysclk = 8000000;
	}

	temp = ((DISC_RCC->RCC_CFGR >> 4) & 0xF);

	if(temp < 8){
		ahbp = 1;
	}
	else{
		ahbp = AHB_PreScaler[temp - 8];
	}

	temp = ((DISC_RCC->RCC_CFGR >> 10) & 0x7);

	if(temp < 4){
		apb1 = 1;
	}
	else{
		apb1 = APB1_PreScaler[temp - 4];
	}
	pclk1 = sysclk / (ahbp * apb1) ;

	return pclk1;
}


