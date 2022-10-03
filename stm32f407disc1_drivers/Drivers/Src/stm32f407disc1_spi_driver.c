

#include "stm32f407disc1_spi_driver.h"

static void SPI_TX_ISR(SPIx_Handle_t *pSPIx_Handle);
static void SPI_RX_ISR(SPIx_Handle_t *pSPIx_Handle);
static void SPI_OVR_ISR(SPIx_Handle_t *pSPIx_Handle);

/*******************************************************************
 * @function					- SPI_PCLKControl
 *
 * @brief						- Enables or disables the peripheral clock of the SPI
 *
 * @param[in]					- Base address of SPI
 * @param[in]					- ENABLE or DISABLE macros
 *
 *
 * @return						- none
 *
 * @Note						- none
 *
 */
void SPI_PCLKControl(SPI_Configuration_t *pSPIx, uint8_t status){
	if(status == ENABLE){
		if(pSPIx==DISC_SPI1){
			DISC_SPI1_PCLK_EN();
		}
		else if(pSPIx==DISC_SPI2I2S2){
			DISC_SPI2_PCLK_EN();
		}
		else if(pSPIx==DISC_SPI3I2S3){
			DISC_SPI3_PCLK_EN();
		}
		else if(pSPIx==DISC_SPI4){
			DISC_SPI4_PCLK_EN();
		}
		else if(pSPIx==DISC_SPI5){
			DISC_SPI5_PCLK_EN();
		}
		else if(pSPIx==DISC_SPI6){
			DISC_SPI6_PCLK_EN();
		}
	}
	else{
		if(pSPIx==DISC_SPI1){
			DISC_SPI1_PCLK_DIS();
		}
		else if(pSPIx==DISC_SPI2I2S2){
			DISC_SPI2_PCLK_DIS();
		}
		else if(pSPIx==DISC_SPI3I2S3){
			DISC_SPI3_PCLK_DIS();
		}
		else if(pSPIx==DISC_SPI4){
			DISC_SPI4_PCLK_DIS();
		}
		else if(pSPIx==DISC_SPI5){
			DISC_SPI5_PCLK_DIS();
		}
		else if(pSPIx==DISC_SPI6){
			DISC_SPI6_PCLK_DIS();
		}
	}
}


/*******************************************************************
 * @function					- SPI_Init
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
void SPI_Init(SPIx_Handle_t *pSPIx_Handle){



	SPI_PCLKControl(pSPIx_Handle->pSPIx, ENABLE);

	//Configure Device Mode
	pSPIx_Handle->pSPIx->SPI_CR1 &= ~(1 << SPIx_REG_MSTR);
	pSPIx_Handle->pSPIx->SPI_CR1 |=  (pSPIx_Handle->SPIx_PinConfig.SPIx_DeviceMode << SPIx_REG_MSTR);

	//Configure Bus Configuration
	if(pSPIx_Handle->SPIx_PinConfig.SPIx_BusConfig == SPIx_BUSCONFG_FULLDUPLEX){
		pSPIx_Handle->pSPIx->SPI_CR1 &= ~(1 << SPIx_REG_BIDIMODE);
	}
	else if (pSPIx_Handle->SPIx_PinConfig.SPIx_BusConfig == SPIx_BUSCONFG_HALFDUPLEX) {
		pSPIx_Handle->pSPIx->SPI_CR1 |= (1 << SPIx_REG_BIDIMODE);
	}
	else if (pSPIx_Handle->SPIx_PinConfig.SPIx_BusConfig == SPIx_BUSCONFG_RXONLY) {
		pSPIx_Handle->pSPIx->SPI_CR1 &= ~(1 << SPIx_REG_BIDIMODE);
		pSPIx_Handle->pSPIx->SPI_CR1 |= (1 << SPIx_REG_RXONLY);
	}

	//Configure PCLK Speed
	pSPIx_Handle->pSPIx->SPI_CR1 &= ~(7 << SPIx_REG_BR);
	pSPIx_Handle->pSPIx->SPI_CR1 |= (pSPIx_Handle->SPIx_PinConfig.SPIx_SCLKSpeed << SPIx_REG_BR);

	//Configure Data Frame
	pSPIx_Handle->pSPIx->SPI_CR1 &= ~(1 << SPIx_REG_DFF);
	pSPIx_Handle->pSPIx->SPI_CR1 |=  pSPIx_Handle->SPIx_PinConfig.SPIx_DFF << SPIx_REG_DFF;

	//Configure Clock Polarity
	pSPIx_Handle->pSPIx->SPI_CR1 &= ~(1 << SPIx_REG_CPOL);
	pSPIx_Handle->pSPIx->SPI_CR1 |=  pSPIx_Handle->SPIx_PinConfig.SPIx_CPOL << SPIx_REG_CPOL;

	//Configure Clock Phase
	pSPIx_Handle->pSPIx->SPI_CR1 &= ~(1 << SPIx_REG_CPHA);
	pSPIx_Handle->pSPIx->SPI_CR1 |=  pSPIx_Handle->SPIx_PinConfig.SPIx_CPHA << SPIx_REG_CPHA;

	//Configure SSM
	pSPIx_Handle->pSPIx->SPI_CR1 &= ~(1 << SPIx_REG_SSM);
	pSPIx_Handle->pSPIx->SPI_CR1 |=  pSPIx_Handle->SPIx_PinConfig.SPIx_SSM << SPIx_REG_SSM;

	//Configure SSI
	if(pSPIx_Handle->SPIx_PinConfig.SPIx_SSM == SPIx_SSM_SSM){
		pSPIx_Handle->pSPIx->SPI_CR1 |= (1 << SPIx_REG_SSI);
	}
	//Configure SSOE
	else if(pSPIx_Handle->SPIx_PinConfig.SPIx_SSM == SPIx_SSM_HSM){
		pSPIx_Handle->pSPIx->SPI_CR2 |= (1 << SPIx_REG_SSOE);
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
void SPI_DeInit(SPI_Configuration_t *pSPIx){
	if(pSPIx==DISC_SPI1){
		DISC_SPI1_PCLK_RST();
	}
	else if(pSPIx==DISC_SPI2I2S2){
		DISC_SPI2_PCLK_RST();
	}
	else if(pSPIx==DISC_SPI3I2S3){
		DISC_SPI3_PCLK_RST();
	}
	else if(pSPIx==DISC_SPI4){
		DISC_SPI4_PCLK_RST();
	}
	else if(pSPIx==DISC_SPI5){
		DISC_SPI5_PCLK_RST();
	}
	else if(pSPIx==DISC_SPI6){
		DISC_SPI6_PCLK_RST();
	}
}

/*******************************************************************
 * @function					- SPI_Status
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
void SPI_Status(SPI_Configuration_t *pSPIx, uint8_t status){

	if(status == ENABLE){
		pSPIx->SPI_CR1 |= (1 << SPIx_REG_SPE);
	}
	else if (status == DISABLE){
		pSPIx->SPI_CR1 &= ~(1 << SPIx_REG_SPE);
	}

}








/*******************************************************************
 * @function					- SPIGetFlagStatus
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
int SPIGetFlagStatus(SPI_Configuration_t *pSPIx, uint8_t flag){

	return pSPIx->SPI_SR & flag;

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
void SPI_SendData(SPI_Configuration_t *pSPIx, uint8_t *pTxBuffer, uint32_t DataLen){

	//Continue sending data until there is 0 bytes
	while(DataLen != 0){

		//Wait until transmision line is clear
		while(! (SPIGetFlagStatus(pSPIx, SPIx_FLAG_TXE)) );

		//Check the dataframe
		if(((pSPIx->SPI_CR1 &  (1 << SPIx_REG_DFF))>>SPIx_REG_DFF) == SPIx_DFF_16BIT){

			//Load the data to the SPI buffer
			pSPIx->SPI_DR = *(uint16_t*)pTxBuffer;

			//Decrease by 1 + 1 bytes
			DataLen =- 2;

			//Increment the address
			(uint16_t*)pTxBuffer ++;
		}
		else if((pSPIx->SPI_CR1 &  (1 << SPIx_REG_DFF)) == SPIx_DFF_8BIT){
			pSPIx->SPI_DR = *pTxBuffer;
			DataLen --;
			pTxBuffer ++;
		}

	}

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
void SPI_RecieveData(SPI_Configuration_t *pSPIx, uint8_t *pRxBuffer, uint32_t DataLen){

	while(DataLen != 0){

		while(! (SPIGetFlagStatus(pSPIx, SPIx_FLAG_RXNE)) );

		if(((pSPIx->SPI_CR1 &  (1 << SPIx_REG_DFF))>>SPIx_REG_DFF) == SPIx_DFF_16BIT){
			*(uint16_t*)pRxBuffer = pSPIx->SPI_DR;
			(uint16_t*)pRxBuffer ++;
			DataLen --;
			DataLen --;
		}
		else if((pSPIx->SPI_CR1 &  (1 << SPIx_REG_DFF)) == SPIx_DFF_8BIT){
			*pRxBuffer = pSPIx->SPI_DR ;
			DataLen --;
			pRxBuffer ++;
		}

	}
}


/*******************************************************************
 * @function					- SPI_SendDataIT
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
uint8_t SPI_SendDataIT(SPIx_Handle_t *pSPIx_Handle, uint8_t *TxBuffer, uint32_t DataLen){

	//Fetch the TX status
	uint8_t status = pSPIx_Handle->TxStat;

	if(status != SPIx_BUSY_TX){

		//Save the data and data length to global variables
		pSPIx_Handle->pTxBuffer = TxBuffer;
		pSPIx_Handle->TxLen = DataLen;

		//Set the busy flag
		pSPIx_Handle->TxStat = SPIx_BUSY_TX;

		//Enable the interrupt
		pSPIx_Handle->pSPIx->SPI_CR2 |= (1 << SPIx_REG_TXEIE);
	}

	return status;
}






/*******************************************************************
 * @function					- SPI_RecieveDataIT
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
uint8_t SPI_RecieveDataIT(SPIx_Handle_t *pSPIx_Handle, uint8_t *RxBuffer, uint32_t DataLen){

	//Fetch the RX status
	uint8_t status = pSPIx_Handle->RxStat;

	if(status != SPIx_BUSY_RX){

		//Save the data and data length to global variables
		pSPIx_Handle->pRxBuffer = RxBuffer;
		pSPIx_Handle->RxLen = DataLen;

		//Set the busy flag
		pSPIx_Handle->RxStat = SPIx_BUSY_RX;

		//Enable the interrupt
		pSPIx_Handle->pSPIx->SPI_CR2 |= (1 << SPIx_REG_RXNEIE);
	}

	return status;
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
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t state){

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
void SPI_IRQHandle(SPIx_Handle_t *pSPIx_Handle){


	uint8_t temp1, temp2;

	//Check TX
	temp1 = pSPIx_Handle->pSPIx->SPI_SR & SPIx_FLAG_TXE;
	temp2 = pSPIx_Handle->pSPIx->SPI_CR2 & (1 << SPIx_REG_TXEIE);

	if(temp1 && temp2){
		SPI_TX_ISR(pSPIx_Handle);
	}

	//Check RX
	temp1 = pSPIx_Handle->pSPIx->SPI_SR & SPIx_FLAG_RXNE;
	temp2 = pSPIx_Handle->pSPIx->SPI_CR2 & (1 << SPIx_REG_RXNEIE);

	if(temp1 && temp2){
		SPI_RX_ISR(pSPIx_Handle);
	}

	//Check ERR Overrun
	temp1 = pSPIx_Handle->pSPIx->SPI_SR & SPIx_FLAG_OVR;
	temp2 = pSPIx_Handle->pSPIx->SPI_CR2 & (1 << SPIx_REG_ERRIE);

	if(temp1 && temp2){
		SPI_OVR_ISR(pSPIx_Handle);
	}
}



/*******************************************************************
 * @function					- SPI_TX_ISR
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
static void SPI_TX_ISR(SPIx_Handle_t *pSPIx_Handle){

	if(((pSPIx_Handle->pSPIx->SPI_CR1 & (1 << SPIx_REG_DFF)) >> SPIx_REG_DFF) == SPIx_DFF_16BIT){
		pSPIx_Handle->pSPIx->SPI_DR = *(uint16_t*)pSPIx_Handle->pTxBuffer;
		pSPIx_Handle->TxLen =- 2;
		(uint16_t*)pSPIx_Handle->pTxBuffer ++;
	}
	else if((pSPIx_Handle->pSPIx->SPI_CR1 & (1 << SPIx_REG_DFF)) == SPIx_DFF_8BIT){
		pSPIx_Handle->pSPIx->SPI_DR = *pSPIx_Handle->pTxBuffer;
		pSPIx_Handle->TxLen --;
		pSPIx_Handle->pTxBuffer ++;
	}
	if(!pSPIx_Handle->TxLen){

		SPI_CloseTransmission(pSPIx_Handle);

		//Custom ISR by user
		SPI_ApplicationCallback(pSPIx_Handle, SPI_EVENT_TX_CMPLT);
	}
}



/*******************************************************************
 * @function					- SPI_RX_ISR
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
static void SPI_RX_ISR(SPIx_Handle_t *pSPIx_Handle){

	if(((pSPIx_Handle->pSPIx->SPI_CR1 &  (1 << SPIx_REG_DFF)) >> SPIx_REG_DFF) == SPIx_DFF_16BIT){
		*(uint16_t*)pSPIx_Handle->pRxBuffer = pSPIx_Handle->pSPIx->SPI_DR;
		pSPIx_Handle->RxLen =- 2;
		(uint16_t*)pSPIx_Handle->pRxBuffer ++;
	}
	else if((pSPIx_Handle->pSPIx->SPI_CR1 & (1 << SPIx_REG_DFF)) == SPIx_DFF_8BIT){
		*pSPIx_Handle->pRxBuffer = pSPIx_Handle->pSPIx->SPI_DR;
		pSPIx_Handle->RxLen --;
		pSPIx_Handle->pRxBuffer ++;
	}
	if(! pSPIx_Handle->RxLen){
		SPI_ApplicationCallback(pSPIx_Handle, SPI_EVENT_RX_CMPLT);
		SPI_CloseReception(pSPIx_Handle);
	}
}



/*******************************************************************
 * @function					- SPI_OVR_ISR
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
static void SPI_OVR_ISR(SPIx_Handle_t *pSPIx_Handle){

	uint8_t temp;
	//Clear ovr flag
	temp = pSPIx_Handle->pSPIx->SPI_DR;
	temp = pSPIx_Handle->pSPIx->SPI_DR;

	(void)temp;
	//inform the application
	SPI_ApplicationCallback(pSPIx_Handle, SPI_EVENT_OVR_ERR);


}



/*******************************************************************
 * @function					- SPI_CloseTransmission
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
void SPI_CloseTransmission(SPIx_Handle_t *pSPIx_Handle){

	//Clear the SPI flags and registers
	pSPIx_Handle->pSPIx->SPI_CR2 &= ~(1 << SPIx_REG_TXEIE);
	pSPIx_Handle->pTxBuffer = NULL;
	pSPIx_Handle->TxStat = SPIx_READY;
	pSPIx_Handle->TxLen = 0;

}















/*******************************************************************
 * @function					- SPI_CloseReception
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
void SPI_CloseReception(SPIx_Handle_t *pSPIx_Handle){

	pSPIx_Handle->pSPIx->SPI_CR2 &= ~(1<<SPIx_REG_RXNEIE);
	pSPIx_Handle->pRxBuffer = NULL;
	pSPIx_Handle->RxStat = SPIx_READY;
	pSPIx_Handle->RxLen = 0;

}











/*******************************************************************
 * @function					- SPI_ApplicationCallback
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
__weak void SPI_ApplicationCallback(SPIx_Handle_t *pSPIx_Handle, uint8_t event){

	// The user needs to fill here
}

