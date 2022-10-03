#ifndef INC_STM32F407DISC1_SPI_H_
#define INC_STM32F407DISC1_SPI_H_

#include "stm32f407disc1.h"

typedef struct{

	uint8_t SPIx_DeviceMode;													/*< Check possible values here: @SPIx_DeviceMode >*/
	uint8_t SPIx_BusConfig;														/*< Check possible values here: @SPIx_BusConfig >*/
	uint8_t SPIx_SCLKSpeed;														/*< Check possible values here: @SPIx_SCLKSpeed >*/
	uint8_t SPIx_DFF;															/*< Check possible values here: @SPIx_DFF >*/
	uint8_t SPIx_CPOL;															/*< Check possible values here: @SPIx_CPOL Function >*/
	uint8_t SPIx_CPHA;															/*< Check possible values here: @SPIx_CPHA Function >*/
	uint8_t SPIx_SSM;															/*< Check possible values here: @SPIx_SSM >*/

}SPIx_PinConfig_t;

typedef struct{

	SPI_Configuration_t *pSPIx;												/*< Holds the base address of the SPIx pin >*/
	SPIx_PinConfig_t SPIx_PinConfig;									    	/*< Holds the configuration settings of the SPIx >*/

	uint8_t	*pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxStat;
	uint8_t RxStat;


}SPIx_Handle_t;

/*
 * @SPIx_DeviceMode
 */

#define SPIx_DEVICEMODE_SLAVE 			0												/*< SPI slave mode >*/
#define SPIx_DEVICEMODE_MASTER  		1												/*< SPI master mode >*/

/*
 * @SPIx_BusConfig
 */

#define SPIx_BUSCONFG_FULLDUPLEX  		1												/*< SPI full-duplex mode >*/
#define SPIx_BUSCONFG_HALFDUPLEX 		2												/*< SPI half-dublex mode >*/
#define SPIx_BUSCONFG_RXONLY 			3												/*< SPI simplex only rx mode >*/

/*
 * @SPIx_SCLKSpeed
 */

#define SPIx_SCLKSPEED_DIV2  		0												/*< SPI input clock divided by 2 >*/
#define SPIx_SCLKSPEED_DIV4  		1												/*< SPI input clock divided by 4 >*/
#define SPIx_SCLKSPEED_DIV8  		2												/*< SPI input clock divided by 8 >*/
#define SPIx_SCLKSPEED_DIV16  		3												/*< SPI input clock divided by 16 >*/
#define SPIx_SCLKSPEED_DIV32  		4												/*< SPI input clock divided by 32 >*/
#define SPIx_SCLKSPEED_DIV64  		5												/*< SPI input clock divided by 64 >*/
#define SPIx_SCLKSPEED_DIV128  		6												/*< SPI input clock divided by 128 >*/
#define SPIx_SCLKSPEED_DIV256  		7												/*< SPI input clock divided by 256 >*/

/*
 * @SPIx_DFF
 */

#define SPIx_DFF_8BIT 			0												/*< SPI 8 bit data frame >*/
#define SPIx_DFF_16BIT  		1												/*< SPI 16 bit data frame >*/


/*
 * @SPIx_CPOL
 */

#define SPIx_CPOL_LEAD 			0												/*< SPI leading edge detection >*/
#define SPIx_CPOL_TRAIL  		1												/*< SPI trailing edge detection >*/


/*
 * @SPIx_CPHA
 */

#define SPIx_CPHA_LOW 			0												/*< SPI clk idle when low >*/
#define SPIx_CPHA_HIGH  		1												/*< SPI clk idle when high >*/


/*
 * @SPIx_SSM
 */

#define SPIx_SSM_HSM 			0												/*< SPI hardware slave manager >*/
#define SPIx_SSM_SSM  			1												/*< SPI software slave manager >*/


/*
 * APIs
 */

// Clock
void SPI_PCLKControl(SPI_Configuration_t *pSPIx, uint8_t status);

//Init
void SPI_Init(SPIx_Handle_t *pSPIx_Handle);
void SPI_DeInit(SPI_Configuration_t *pSPIx);
void SPI_Status(SPI_Configuration_t *pSPIx, uint8_t status);

//Data send & recieve
void SPI_SendData(SPI_Configuration_t *pSPIx, uint8_t *TxBuffer, uint32_t DataLen);
void SPI_RecieveData(SPI_Configuration_t *pSPIx, uint8_t *RxBuffer, uint32_t DataLen);

uint8_t SPI_SendDataIT(SPIx_Handle_t *pSPIx_Handle, uint8_t *TxBuffer, uint32_t DataLen);
uint8_t SPI_RecieveDataIT(SPIx_Handle_t *pSPIx_Handle, uint8_t *RxBuffer, uint32_t DataLen);

//IRQ
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t state);
void SPI_IRQHandle(SPIx_Handle_t *pSPIx_Handle);

//MISC
int SPIGetFlagStatus(SPI_Configuration_t *pSPIx, uint8_t flag);
void SPI_Status(SPI_Configuration_t *pSPIx, uint8_t status);
void SPI_ClearOVRFlag(SPI_Configuration_t *pSPIx);
void SPI_CloseTransmission(SPIx_Handle_t *pSPIx_Handle);
void SPI_CloseReception(SPIx_Handle_t *pSPIx_Handle);
void SPI_ApplicationCallback(SPIx_Handle_t *pSPIx_Handle, uint8_t event);

/*
 * OTHER MACROS
 */

#define SPIx_REG_BIDIMODE					15
#define SPIx_REG_RXONLY 					10
#define SPIx_REG_BR 						3
#define SPIx_REG_DFF 						11
#define SPIx_REG_CPOL 						1
#define SPIx_REG_CPHA						0
#define SPIx_REG_SSM						9
#define SPIx_REG_MSTR						2
#define SPIx_REG_SPE						6
#define SPIx_REG_SSI						8
#define SPIx_REG_SSOE 						2
#define SPIx_REG_TXEIE						7
#define SPIx_REG_RXNEIE 					6
#define SPIx_REG_ERRIE 						5

#define SPIx_FLAG_BSY 						(1 << 7)
#define SPIx_FLAG_OVR						(1 << 6)
#define SPIx_FLAG_MODF						(1 << 5)
#define SPIx_FLAG_CRCERR					(1 << 4)
#define SPIx_FLAG_UDR 						(1 << 3)
#define SPIx_FLAG_CHSIDE					(1 << 2)
#define SPIx_FLAG_TXE						(1 << 1)
#define SPIx_FLAG_RXNE						(1 << 0)

#define SPIx_READY							0
#define SPIx_BUSY_RX						1
#define SPIx_BUSY_TX						2

#define SPI_EVENT_TX_CMPLT					1
#define SPI_EVENT_RX_CMPLT					2
#define SPI_EVENT_OVR_ERR					3

#endif /* INC_STM32F407DISC1_SPI_H_ */
