#ifndef __SD_SPI_H
#define __SD_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/*
 * Configure these names to match your CubeMX-generated handles/pins.
 *
 * Example:
 *   SPI_HandleTypeDef hspi1;
 *   #define SD_SPI_HANDLE hspi1
 *
 *   #define SD_CS_GPIO_Port GPIOB
 *   #define SD_CS_Pin       GPIO_PIN_0
 */

extern SPI_HandleTypeDef hspi3;

#define SD_SPI_HANDLE      hspi3

#ifndef SD_CS_GPIO_Port
#error "Define SD_CS_GPIO_Port in main.h or before including sd_spi.h"
#endif

#ifndef SD_CS_Pin
#error "Define SD_CS_Pin in main.h or before including sd_spi.h"
#endif

typedef enum
{
    SD_SPI_OK = 0,
    SD_SPI_ERROR = -1,
    SD_SPI_TIMEOUT = -2
} SD_SPI_Status;

/* Public API used by FatFs user_diskio.c */
int SD_SPI_Init(void);
int SD_SPI_ReadBlocks(uint8_t *buff, uint32_t sector, uint32_t count);
int SD_SPI_WriteBlocks(const uint8_t *buff, uint32_t sector, uint32_t count);
uint32_t SD_SPI_GetSectorCount(void);
void SD_SPI_ReleaseBus(void);

#ifdef __cplusplus
}
#endif

#endif /* __SD_SPI_H */
