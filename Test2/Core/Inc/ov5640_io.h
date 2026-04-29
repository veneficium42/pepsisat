/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef OV5640_IO_H
#define OV5640_IO_H

#include <stddef.h>
#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

//=======================================================================================================
//												VARIABLES
//=======================================================================================================
extern I2C_HandleTypeDef hi2c1;

//=======================================================================================================
//												FUNCTIONS
//=======================================================================================================
int32_t OV5640_IO_ReadReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length);
int32_t OV5640_IO_WriteReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length);

/**
  * @}
  */
#ifdef __cplusplus
}
#endif

#endif /* OV5640_REG_H */
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
