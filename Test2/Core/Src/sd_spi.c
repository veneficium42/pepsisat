#include "sd_spi.h"

/*
 * Minimal SD-card SPI-mode driver for FatFs.
 *
 * Required SPI settings:
 * - SPI mode 0: CPOL = 0, CPHA = 0
 * - NSS software
 * - NSS pulse disabled
 * - MasterKeepIOState enabled on STM32H7
 * - Slow clock during initialization, about 200–400 kHz
 */

#define SD_SPI_DUMMY_BYTE      0xFFU
#define SD_SPI_BLOCK_SIZE      512U
#define SD_SPI_TIMEOUT_MS      5000U

/* SD commands: command index only. 0x40 is added in SD_SendCommand(). */
#define CMD0    (0U)
#define CMD1    (1U)
#define CMD8    (8U)
#define CMD9    (9U)
#define CMD10   (10U)
#define CMD12   (12U)
#define CMD16   (16U)
#define CMD17   (17U)
#define CMD18   (18U)
#define CMD24   (24U)
#define CMD25   (25U)
#define CMD55   (55U)
#define CMD58   (58U)
#define ACMD41  (41U)

#define TOKEN_START_BLOCK_SINGLE   0xFEU
#define TOKEN_START_BLOCK_MULTI    0xFCU
#define TOKEN_STOP_TRAN            0xFDU

#define DATA_ACCEPTED              0x05U

static uint8_t sd_is_initialized = 0U;
static uint8_t sd_is_sdhc = 0U;
static uint32_t sd_sector_count = 0U;

/* Debug variables */
volatile uint32_t dbg_read_sector = 0;
volatile uint32_t dbg_read_count = 0;
volatile uint32_t dbg_read_addr = 0;
volatile uint8_t  dbg_read_cmd_r = 0xFF;
volatile uint8_t  dbg_read_result = 0xFF;
volatile uint8_t  dbg_is_sdhc = 0;

volatile uint8_t  dbg_first_token = 0xFF;
volatile uint8_t  dbg_final_token = 0xFF;
volatile uint32_t dbg_token_retry = 0;
volatile uint8_t  dbg_receive_result = 0xFF;

volatile uint8_t dbg_cmd58_r = 0xFF;
volatile uint8_t dbg_ocr0 = 0xFF;
volatile uint8_t dbg_ocr1 = 0xFF;
volatile uint8_t dbg_ocr2 = 0xFF;
volatile uint8_t dbg_ocr3 = 0xFF;

volatile HAL_StatusTypeDef dbg_spi_status = HAL_OK;
volatile uint8_t dbg_spi_tx = 0xFF;
volatile uint8_t dbg_spi_rx = 0xFF;

static void SD_CS_Low(void)
{
    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);
}


static void SD_CS_High(void)
{
    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
}


static uint8_t SPI_TxRx(uint8_t data)
{
    uint8_t rx = 0xFFU;

    dbg_spi_tx = data;

    dbg_spi_status = HAL_SPI_TransmitReceive(&SD_SPI_HANDLE,
                                             &data,
                                             &rx,
                                             1U,
                                             SD_SPI_TIMEOUT_MS);

    dbg_spi_rx = rx;

    if (dbg_spi_status != HAL_OK)
    {
        return 0xFFU;
    }

    return rx;
}


static void SPI_SendDummyClocks(uint32_t count)
{
    for (uint32_t i = 0; i < count; i++)
    {
        (void)SPI_TxRx(SD_SPI_DUMMY_BYTE);
    }
}


static uint8_t SD_WaitReady(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    uint8_t r;

    do
    {
        r = SPI_TxRx(SD_SPI_DUMMY_BYTE);

        if (r == 0xFFU)
        {
            return 1U;
        }

    } while ((HAL_GetTick() - start) < timeout_ms);

    return 0U;
}


static uint8_t SD_SendCommand(uint8_t cmd, uint32_t arg)
{
    uint8_t crc;
    uint8_t response = 0xFFU;

    SD_CS_High();
    SPI_TxRx(SD_SPI_DUMMY_BYTE);

    SD_CS_Low();
    SPI_TxRx(SD_SPI_DUMMY_BYTE);

    /*
     * For normal commands after initialization, wait until card is ready.
     * For CMD0 this is not required.
     */
    if (cmd != CMD0)
    {
        if (!SD_WaitReady(SD_SPI_TIMEOUT_MS))
        {
            return 0xFFU;
        }
    }

    SPI_TxRx((uint8_t)(0x40U | cmd));
    SPI_TxRx((uint8_t)(arg >> 24));
    SPI_TxRx((uint8_t)(arg >> 16));
    SPI_TxRx((uint8_t)(arg >> 8));
    SPI_TxRx((uint8_t)arg);

    crc = 0x01U;

    if (cmd == CMD0)
    {
        crc = 0x95U;
    }
    else if (cmd == CMD8)
    {
        crc = 0x87U;
    }

    SPI_TxRx(crc);

    for (uint32_t i = 0; i < 10U; i++)
    {
        response = SPI_TxRx(SD_SPI_DUMMY_BYTE);

        if ((response & 0x80U) == 0U)
        {
            return response;
        }
    }

    return response;
}


static uint8_t SD_ReceiveDataBlock(uint8_t *buff, uint32_t len)
{
    uint8_t token = 0xFFU;
    uint32_t start = HAL_GetTick();
    uint32_t retry = 0U;

    dbg_first_token = 0xFFU;
    dbg_final_token = 0xFFU;
    dbg_token_retry = 0U;
    dbg_receive_result = 0xAAU;

    do
    {
        token = SPI_TxRx(SD_SPI_DUMMY_BYTE);

        if (retry == 0U)
        {
            dbg_first_token = token;
        }

        retry++;
        dbg_final_token = token;
        dbg_token_retry = retry;

        if (token == TOKEN_START_BLOCK_SINGLE)
        {
            dbg_receive_result = 1U;
            break;
        }

    } while ((HAL_GetTick() - start) < SD_SPI_TIMEOUT_MS);

    if (token != TOKEN_START_BLOCK_SINGLE)
    {
        dbg_receive_result = 2U;
        return 0U;
    }

    for (uint32_t i = 0; i < len; i++)
    {
        buff[i] = SPI_TxRx(SD_SPI_DUMMY_BYTE);
    }

    /* Discard CRC */
    SPI_TxRx(SD_SPI_DUMMY_BYTE);
    SPI_TxRx(SD_SPI_DUMMY_BYTE);

    return 1U;
}


static uint8_t SD_TransmitDataBlock(const uint8_t *buff, uint8_t token)
{
    uint8_t resp;

    if (!SD_WaitReady(SD_SPI_TIMEOUT_MS))
    {
        return 0U;
    }

    SPI_TxRx(token);

    if (token == TOKEN_STOP_TRAN)
    {
        return 1U;
    }

    for (uint32_t i = 0; i < SD_SPI_BLOCK_SIZE; i++)
    {
        SPI_TxRx(buff[i]);
    }

    /* Dummy CRC */
    SPI_TxRx(SD_SPI_DUMMY_BYTE);
    SPI_TxRx(SD_SPI_DUMMY_BYTE);

    resp = SPI_TxRx(SD_SPI_DUMMY_BYTE);

    if ((resp & 0x1FU) != DATA_ACCEPTED)
    {
        return 0U;
    }

    return 1U;
}


static uint32_t SD_GetCSDCapacity(void)
{
    uint8_t csd[16];

    if (SD_SendCommand(CMD9, 0U) != 0U)
    {
        SD_CS_High();
        SPI_TxRx(SD_SPI_DUMMY_BYTE);
        return 0U;
    }

    if (!SD_ReceiveDataBlock(csd, 16U))
    {
        SD_CS_High();
        SPI_TxRx(SD_SPI_DUMMY_BYTE);
        return 0U;
    }

    SD_CS_High();
    SPI_TxRx(SD_SPI_DUMMY_BYTE);

    if ((csd[0] >> 6) == 1U)
    {
        uint32_t c_size;

        c_size = ((uint32_t)(csd[7] & 0x3FU) << 16) |
                 ((uint32_t)csd[8] << 8) |
                 ((uint32_t)csd[9]);

        return (c_size + 1U) * 1024U;
    }
    else
    {
        uint32_t read_bl_len;
        uint32_t c_size;
        uint32_t c_size_mult;
        uint32_t block_len;
        uint32_t mult;
        uint32_t block_count;
        uint32_t capacity_bytes;

        read_bl_len = csd[5] & 0x0FU;

        c_size = ((uint32_t)(csd[6] & 0x03U) << 10) |
                 ((uint32_t)csd[7] << 2) |
                 ((uint32_t)(csd[8] & 0xC0U) >> 6);

        c_size_mult = ((uint32_t)(csd[9] & 0x03U) << 1) |
                      ((uint32_t)(csd[10] & 0x80U) >> 7);

        block_len = 1UL << read_bl_len;
        mult = 1UL << (c_size_mult + 2U);
        block_count = (c_size + 1U) * mult;
        capacity_bytes = block_count * block_len;

        return capacity_bytes / SD_SPI_BLOCK_SIZE;
    }
}


int SD_SPI_Init(void)
{
    uint8_t r;
    uint8_t ocr[4];

    sd_is_initialized = 0U;
    sd_is_sdhc = 0U;
    sd_sector_count = 0U;

    SD_CS_High();
    SPI_SendDummyClocks(10U);

    /* CMD0: enter SPI idle mode */
    r = SD_SendCommand(CMD0, 0U);
    SD_CS_High();
    SPI_TxRx(SD_SPI_DUMMY_BYTE);

    if (r != 0x01U)
    {
        return SD_SPI_ERROR;
    }

    /* CMD8: SD v2 voltage check */
    r = SD_SendCommand(CMD8, 0x000001AAU);

    if (r == 0x01U)
    {
        ocr[0] = SPI_TxRx(SD_SPI_DUMMY_BYTE);
        ocr[1] = SPI_TxRx(SD_SPI_DUMMY_BYTE);
        ocr[2] = SPI_TxRx(SD_SPI_DUMMY_BYTE);
        ocr[3] = SPI_TxRx(SD_SPI_DUMMY_BYTE);

        SD_CS_High();
        SPI_TxRx(SD_SPI_DUMMY_BYTE);

        if ((ocr[2] != 0x01U) || (ocr[3] != 0xAAU))
        {
            return SD_SPI_ERROR;
        }

        uint32_t start = HAL_GetTick();

        do
        {
            uint8_t r55;
            uint8_t r41;

            r55 = SD_SendCommand(CMD55, 0U);
            SD_CS_High();
            SPI_TxRx(SD_SPI_DUMMY_BYTE);

            if (r55 > 0x01U)
            {
                r = r55;
                break;
            }

            r41 = SD_SendCommand(ACMD41, 0x40000000U);
            SD_CS_High();
            SPI_TxRx(SD_SPI_DUMMY_BYTE);

            r = r41;

            if (r == 0x00U)
            {
                break;
            }

        } while ((HAL_GetTick() - start) < SD_SPI_TIMEOUT_MS);

        if (r != 0x00U)
        {
            return SD_SPI_TIMEOUT;
        }

        r = SD_SendCommand(CMD58, 0U);
        dbg_cmd58_r = r;

        if (r != 0x00U)
        {
            SD_CS_High();
            SPI_TxRx(SD_SPI_DUMMY_BYTE);
            return SD_SPI_ERROR;
        }

        ocr[0] = SPI_TxRx(SD_SPI_DUMMY_BYTE);
        ocr[1] = SPI_TxRx(SD_SPI_DUMMY_BYTE);
        ocr[2] = SPI_TxRx(SD_SPI_DUMMY_BYTE);
        ocr[3] = SPI_TxRx(SD_SPI_DUMMY_BYTE);

        SD_CS_High();
        SPI_TxRx(SD_SPI_DUMMY_BYTE);

        dbg_ocr0 = ocr[0];
        dbg_ocr1 = ocr[1];
        dbg_ocr2 = ocr[2];
        dbg_ocr3 = ocr[3];

        if (ocr[0] & 0x40U)
        {
            sd_is_sdhc = 1U;
        }
    }
    else
    {
        /* Old SDSC/MMC fallback */
        SD_CS_High();
        SPI_TxRx(SD_SPI_DUMMY_BYTE);

        uint32_t start = HAL_GetTick();

        do
        {
            uint8_t r55;
            uint8_t r41;

            r55 = SD_SendCommand(CMD55, 0U);
            SD_CS_High();
            SPI_TxRx(SD_SPI_DUMMY_BYTE);

            if (r55 <= 0x01U)
            {
                r41 = SD_SendCommand(ACMD41, 0U);
                SD_CS_High();
                SPI_TxRx(SD_SPI_DUMMY_BYTE);

                r = r41;

                if (r == 0x00U)
                {
                    break;
                }
            }
            else
            {
                r = r55;
            }

        } while ((HAL_GetTick() - start) < SD_SPI_TIMEOUT_MS);

        if (r != 0x00U)
        {
            start = HAL_GetTick();

            do
            {
                r = SD_SendCommand(CMD1, 0U);
                SD_CS_High();
                SPI_TxRx(SD_SPI_DUMMY_BYTE);

                if (r == 0x00U)
                {
                    break;
                }

            } while ((HAL_GetTick() - start) < SD_SPI_TIMEOUT_MS);
        }

        if (r != 0x00U)
        {
            return SD_SPI_TIMEOUT;
        }

        if (SD_SendCommand(CMD16, SD_SPI_BLOCK_SIZE) != 0U)
        {
            SD_CS_High();
            SPI_TxRx(SD_SPI_DUMMY_BYTE);
            return SD_SPI_ERROR;
        }

        SD_CS_High();
        SPI_TxRx(SD_SPI_DUMMY_BYTE);
    }

    sd_sector_count = SD_GetCSDCapacity();

    /*
     * Capacity reading is useful but should not prevent basic mount.
     * Some FatFs configurations may not immediately need sector count.
     */
    sd_is_initialized = 1U;

    return SD_SPI_OK;
}


int SD_SPI_ReadBlocks(uint8_t *buff, uint32_t sector, uint32_t count)
{
    if ((!sd_is_initialized) || (buff == 0) || (count == 0U))
    {
        dbg_read_result = 10U;
        return SD_SPI_ERROR;
    }

    uint32_t addr = sd_is_sdhc ? sector : (sector * SD_SPI_BLOCK_SIZE);

    dbg_read_sector = sector;
    dbg_read_count = count;
    dbg_read_addr = addr;
    dbg_is_sdhc = sd_is_sdhc;

    if (count == 1U)
    {
        dbg_read_cmd_r = SD_SendCommand(CMD17, addr);

        if (dbg_read_cmd_r != 0U)
        {
            SD_CS_High();
            SPI_TxRx(SD_SPI_DUMMY_BYTE);
            dbg_read_result = 1U;
            return SD_SPI_ERROR;
        }

        if (!SD_ReceiveDataBlock(buff, SD_SPI_BLOCK_SIZE))
        {
            SD_CS_High();
            SPI_TxRx(SD_SPI_DUMMY_BYTE);
            dbg_read_result = 2U;
            return SD_SPI_ERROR;
        }

        SD_CS_High();
        SPI_TxRx(SD_SPI_DUMMY_BYTE);

        dbg_read_result = 0U;
        return SD_SPI_OK;
    }
    else
    {
        dbg_read_cmd_r = SD_SendCommand(CMD18, addr);

        if (dbg_read_cmd_r != 0U)
        {
            SD_CS_High();
            SPI_TxRx(SD_SPI_DUMMY_BYTE);
            dbg_read_result = 3U;
            return SD_SPI_ERROR;
        }

        for (uint32_t i = 0; i < count; i++)
        {
            if (!SD_ReceiveDataBlock(buff + (i * SD_SPI_BLOCK_SIZE), SD_SPI_BLOCK_SIZE))
            {
                SD_CS_High();
                SPI_TxRx(SD_SPI_DUMMY_BYTE);
                dbg_read_result = 4U;
                return SD_SPI_ERROR;
            }
        }

        (void)SD_SendCommand(CMD12, 0U);
        SD_CS_High();
        SPI_TxRx(SD_SPI_DUMMY_BYTE);

        dbg_read_result = 0U;
        return SD_SPI_OK;
    }
}


int SD_SPI_WriteBlocks(const uint8_t *buff, uint32_t sector, uint32_t count)
{
    if ((!sd_is_initialized) || (buff == 0) || (count == 0U))
    {
        return SD_SPI_ERROR;
    }

    uint32_t addr = sd_is_sdhc ? sector : (sector * SD_SPI_BLOCK_SIZE);

    if (count == 1U)
    {
        if (SD_SendCommand(CMD24, addr) != 0U)
        {
            SD_CS_High();
            SPI_TxRx(SD_SPI_DUMMY_BYTE);
            return SD_SPI_ERROR;
        }

        if (!SD_TransmitDataBlock(buff, TOKEN_START_BLOCK_SINGLE))
        {
            SD_CS_High();
            SPI_TxRx(SD_SPI_DUMMY_BYTE);
            return SD_SPI_ERROR;
        }

        SD_CS_High();
        SPI_TxRx(SD_SPI_DUMMY_BYTE);

        return SD_SPI_OK;
    }
    else
    {
        static const uint8_t dummy_block[SD_SPI_BLOCK_SIZE] = {0};

        if (SD_SendCommand(CMD25, addr) != 0U)
        {
            SD_CS_High();
            SPI_TxRx(SD_SPI_DUMMY_BYTE);
            return SD_SPI_ERROR;
        }

        for (uint32_t i = 0; i < count; i++)
        {
            if (!SD_TransmitDataBlock(buff + (i * SD_SPI_BLOCK_SIZE), TOKEN_START_BLOCK_MULTI))
            {
                SD_CS_High();
                SPI_TxRx(SD_SPI_DUMMY_BYTE);
                return SD_SPI_ERROR;
            }
        }

        if (!SD_TransmitDataBlock(dummy_block, TOKEN_STOP_TRAN))
        {
            SD_CS_High();
            SPI_TxRx(SD_SPI_DUMMY_BYTE);
            return SD_SPI_ERROR;
        }

        SD_CS_High();
        SPI_TxRx(SD_SPI_DUMMY_BYTE);

        return SD_SPI_OK;
    }
}


uint32_t SD_SPI_GetSectorCount(void)
{
    return sd_sector_count;
}

void SD_SPI_ReleaseBus(void)
{
    SD_CS_High();
    SPI_TxRx(0xFF);
}

