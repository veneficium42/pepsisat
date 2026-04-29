#include <ov5640_io.h>

int32_t OV5640_IO_ReadReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
	// Use the DevAddr parameter, shifted left by 1 for HAL I2C functions
    return HAL_I2C_Mem_Read(&hi2c1, (DevAddr << 1),
    						Reg,
							I2C_MEMADD_SIZE_16BIT,
							pData,
							Length,
							HAL_MAX_DELAY);
}

int32_t OV5640_IO_WriteReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
	// Use the DevAddr parameter, shifted left by 1 for HAL I2C functions
    return HAL_I2C_Mem_Write(&hi2c1, (DevAddr << 1),
    						Reg,
                            I2C_MEMADD_SIZE_16BIT,
                            pData,
							Length,
							HAL_MAX_DELAY);
}
