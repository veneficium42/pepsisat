/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>

#include "stm32h7xx_hal_qspi.h"

#include "ov5640.h"
#include "ov5640_io.h"
#include "usb_io.h"
#include "sd_spi.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// _write redefinition for printf()
int _write(int file, char *ptr, int len)
{
    for (int i = 0; i < len; i++)
    {
        ITM_SendChar((*ptr++));
    }
    return len;
}

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

DCMI_HandleTypeDef hdcmi;
DMA_HandleTypeDef hdma_dcmi;

I2C_HandleTypeDef hi2c1;

QSPI_HandleTypeDef hqspi;

RNG_HandleTypeDef hrng;

SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart5;

DMA_HandleTypeDef hdma_memtomem_dma1_stream0;
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for cameraTask */
osThreadId_t cameraTaskHandle;
const osThreadAttr_t cameraTask_attributes = {
  .name = "cameraTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* USER CODE BEGIN PV */

//OV5640_Object_t camera;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DCMI_Init(void);
static void MX_I2C1_Init(void);
static void MX_RNG_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_UART5_Init(void);
static void MX_SPI3_Init(void);
void StartDefaultTask(void *argument);
void StartCameraTask(void *argument);

/* USER CODE BEGIN PFP */

static int32_t I2C_Init(void){return 0;}
static int32_t I2C_DeInit(void){return 0;}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//#define WIDTH  160
//#define HEIGHT 120

//define WIDTH  800
//#define HEIGHT 480

#define WIDTH  640
#define HEIGHT 480

//#define CSIZE  2 // RGB565
#define CSIZE  1 // Y8

#define FRAME_BYTES (WIDTH * HEIGHT * CSIZE)
#define FRAME_WORDS (FRAME_BYTES / 4)


__attribute__((section(".RAM_D1"), aligned(32)))
uint8_t frame_buffer[FRAME_BYTES];

//__attribute__((section(".qspi_ram")))
//volatile uint8_t qspi_buffer[8192];


volatile uint32_t dbg_tick_before;
volatile uint32_t dbg_tick_after;
volatile uint32_t dbg_primask;
volatile uint32_t dbg_basepri;
volatile uint32_t dbg_ipsr;
volatile FRESULT dbg_sd_res;

extern FATFS SDFatFS;
extern char SDPath[4];

//DCMI frame callback
void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi)
{
	__NOP();

	// Suspend DCMI
	HAL_DCMI_Suspend(hdcmi);

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Sends a direct-to-task notification to cameraTaskHandle
    vTaskNotifyGiveFromISR(cameraTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// PSRAM_EnableQuadMode
HAL_StatusTypeDef PSRAM_EnableQuadMode()
{
	QSPI_CommandTypeDef cmd = {0};

	cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	cmd.Instruction     = 0x35; // Enter Quad mode
	cmd.AddressMode     = QSPI_ADDRESS_NONE;
	cmd.DataMode        = QSPI_DATA_NONE;
	cmd.DummyCycles     = 0;

	if (HAL_QSPI_Command(&hqspi, &cmd, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		return HAL_ERROR;
	}

    return HAL_OK;
}

// PSRAM_Write
HAL_StatusTypeDef PSRAM_Write(uint32_t addr, uint8_t *data, uint32_t size)
{
    QSPI_CommandTypeDef cmd = {0};

    cmd.InstructionMode = QSPI_INSTRUCTION_4_LINES;
    cmd.AddressMode     = QSPI_ADDRESS_4_LINES;
    cmd.DataMode        = QSPI_DATA_4_LINES;

    cmd.AddressSize     = QSPI_ADDRESS_24_BITS;
    cmd.Instruction     = 0x02;  // QUAD WRITE (APS6404L)
    cmd.Address         = addr;
    cmd.NbData          = size;

    cmd.DummyCycles     = 0;

    if (HAL_QSPI_Command(&hqspi, &cmd, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    	return HAL_ERROR;

    if (HAL_QSPI_Transmit(&hqspi, data, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    	return HAL_ERROR;

    return HAL_OK;
}

// PSRAM_Read
HAL_StatusTypeDef PSRAM_Read(uint32_t addr, uint8_t *data, uint32_t size)
{
    QSPI_CommandTypeDef cmd = {0};

    cmd.InstructionMode = QSPI_INSTRUCTION_4_LINES;
    cmd.AddressMode     = QSPI_ADDRESS_4_LINES;
    cmd.DataMode        = QSPI_DATA_4_LINES;

    cmd.AddressSize     = QSPI_ADDRESS_24_BITS;
    cmd.Instruction     = 0x0B;  // QUAD READ (APS6404L)
    cmd.Address         = addr;
    cmd.NbData          = size;

    cmd.DummyCycles     = 4;

    if (HAL_QSPI_Command(&hqspi, &cmd, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    	return HAL_ERROR;

    if (HAL_QSPI_Receive(&hqspi, data, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    	return HAL_ERROR;

    return HAL_OK;
}

#define NUM_BARS 16

// FillGrayRamp_Y8
void FillGrayRamp_Y8(uint8_t *buf, uint32_t width, uint32_t height)
{
    for (uint32_t y = 0; y < height; y++)
    {
        for (uint32_t x = 0; x < width; x++)
        {
            uint8_t gray = (uint8_t)((x * 255) / (width - 1));
            buf[y * width + x] = gray;
        }
    }
}

// SD_TestWrite
static void SD_TestWrite(void)
{
    FRESULT res;
    FIL file;
    UINT written;

    const char text[] = "STM32H753 SPI SD test\r\n";

    res = f_mount(&USERFatFS, USERPath, 1);
    if (res != FR_OK)
    {
    	SD_SPI_ReleaseBus();
        Error_Handler();
    }

    res = f_open(&file, "test.txt", FA_CREATE_ALWAYS | FA_WRITE);
    if (res != FR_OK)
    {
    	SD_SPI_ReleaseBus();
    	SD_SPI_ReleaseBus();
        Error_Handler();
    }

    res = f_write(&file, text, strlen(text), &written);
    if ((res != FR_OK) || (written != strlen(text)))
    {
        f_close(&file);
        SD_SPI_ReleaseBus();
        Error_Handler();
    }

    f_close(&file);

    SD_SPI_ReleaseBus();
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DCMI_Init();
  MX_I2C1_Init();
  MX_RNG_Init();
  MX_FATFS_Init();
  MX_QUADSPI_Init();
  MX_UART5_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */

 SD_TestWrite();

  dbg_basepri = __get_BASEPRI();   // should show 0x50 now

  __set_BASEPRI(0);                // temporary recovery test

  dbg_basepri = __get_BASEPRI();   // should become 0

  HAL_Delay(100);

  dbg_tick_after = HAL_GetTick();

  // Enable Quad SPI mode
  if(PSRAM_EnableQuadMode(&hqspi) != OV5640_OK)
  {
	  Error_Handler();
  }

  OV5640_Object_t camera;
  OV5640_IO_t io_ctx;

  // Initialize camera object
  camera.IsInitialized	= 0U;
  camera.Mode			= PARALLEL_MODE;

  // Bind io_ctx
  io_ctx.Init	  = I2C_Init;
  io_ctx.DeInit	  = I2C_DeInit;
  io_ctx.Address  = 0x3C;
  io_ctx.ReadReg  = OV5640_IO_ReadReg;
  io_ctx.WriteReg = OV5640_IO_WriteReg;
  io_ctx.GetTick  = (long int (*)(void))HAL_GetTick;

  // Reset Camera
  HAL_GPIO_WritePin(GPIOD, CAMERA_RESET_Pin, GPIO_PIN_RESET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(GPIOD, CAMERA_RESET_Pin, GPIO_PIN_SET);
  HAL_Delay(100);

  // Register Camera IO
  OV5640_RegisterBusIO(&camera, &io_ctx);

  // Check camera ID
  uint32_t id;
  OV5640_ReadID(&camera, &id);

  // Init camera device (Y8 Data Format)
  if (id == OV5640_ID)
  {
	  //if(OV5640_Init(&camera, OV5640_R160x120, OV5640_Y8) != OV5640_OK)
	  //if(OV5640_Init(&camera, OV5640_R480x272, OV5640_RGB565) != OV5640_OK)
	  //if(OV5640_Init(&camera, OV5640_R480x272, OV5640_RGB565) != OV5640_OK)
	  if(OV5640_Init(&camera, OV5640_R640x480, OV5640_Y8) != OV5640_OK)
	  {
		  // something is wrong (I2C, wiring, clock)
		  HAL_GPIO_WritePin(GPIOB, LED3_Pin, GPIO_PIN_SET);
		  for (;;);
	  }
  }
  else
  {
      // something is wrong (I2C, wiring, clock)
	  HAL_GPIO_WritePin(GPIOB, LED3_Pin, GPIO_PIN_SET);
	  for (;;);
  }

  // Enable colorbar mode
  //OV5640_ColorbarModeConfig(&camera, COLORBAR_MODE_ENABLE);

  // Disable ACG
  //uint8_t tmp = 0x03;
  //ov5640_write_reg(&camera.Ctx, OV5640_AEC_PK_MANUAL, &tmp, 1);

  /* Wait 1s to let auto-loops in the camera module converge and lead to correct exposure */
  HAL_Delay(1000);
  
  // Start camera
  OV5640_Start(&camera);

  // Start DMA continuous mode
  if(HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_CONTINUOUS, (uint32_t)frame_buffer, FRAME_WORDS) != HAL_OK)
  {
	  Error_Handler();
  }

  // Enable ITM stimulus port 0
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  ITM->TCR |= ITM_TCR_ITMENA_Msk;
  ITM->TER |= (1 << 0);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of cameraTask */
  cameraTaskHandle = osThreadNew(StartCameraTask, NULL, &cameraTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DCMI Initialization Function
  * @param None
  * @retval None
  */
static void MX_DCMI_Init(void)
{

  /* USER CODE BEGIN DCMI_Init 0 */

  /* USER CODE END DCMI_Init 0 */

  /* USER CODE BEGIN DCMI_Init 1 */

  /* USER CODE END DCMI_Init 1 */
  hdcmi.Instance = DCMI;
  hdcmi.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
  hdcmi.Init.PCKPolarity = DCMI_PCKPOLARITY_RISING;
  hdcmi.Init.VSPolarity = DCMI_VSPOLARITY_HIGH;
  hdcmi.Init.HSPolarity = DCMI_HSPOLARITY_HIGH;
  hdcmi.Init.CaptureRate = DCMI_CR_ALL_FRAME;
  hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
  hdcmi.Init.JPEGMode = DCMI_JPEG_DISABLE;
  hdcmi.Init.ByteSelectMode = DCMI_BSM_ALL;
  hdcmi.Init.ByteSelectStart = DCMI_OEBS_ODD;
  hdcmi.Init.LineSelectMode = DCMI_LSM_ALL;
  hdcmi.Init.LineSelectStart = DCMI_OELS_ODD;
  if (HAL_DCMI_Init(&hdcmi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DCMI_Init 2 */

  /* USER CODE END DCMI_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10C0ECFF;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 7;
  hqspi.Init.FifoThreshold = 16;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
  hqspi.Init.FlashSize = 22;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  hqspi.Init.FlashID = QSPI_FLASH_ID_1;
  hqspi.Init.DualFlash = QSPI_DUALFLASH_DISABLE;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  hrng.Init.ClockErrorDetection = RNG_CED_ENABLE;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x0;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 1843200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * Enable DMA controller clock
  * Configure DMA for memory to memory transfers
  *   hdma_memtomem_dma1_stream0
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* Configure DMA request hdma_memtomem_dma1_stream0 on DMA1_Stream0 */
  hdma_memtomem_dma1_stream0.Instance = DMA1_Stream0;
  hdma_memtomem_dma1_stream0.Init.Request = DMA_REQUEST_MEM2MEM;
  hdma_memtomem_dma1_stream0.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma1_stream0.Init.PeriphInc = DMA_PINC_ENABLE;
  hdma_memtomem_dma1_stream0.Init.MemInc = DMA_MINC_ENABLE;
  hdma_memtomem_dma1_stream0.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_memtomem_dma1_stream0.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdma_memtomem_dma1_stream0.Init.Mode = DMA_NORMAL;
  hdma_memtomem_dma1_stream0.Init.Priority = DMA_PRIORITY_LOW;
  hdma_memtomem_dma1_stream0.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
  hdma_memtomem_dma1_stream0.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  hdma_memtomem_dma1_stream0.Init.MemBurst = DMA_MBURST_INC4;
  hdma_memtomem_dma1_stream0.Init.PeriphBurst = DMA_PBURST_INC4;
  if (HAL_DMA_Init(&hdma_memtomem_dma1_stream0) != HAL_OK)
  {
    Error_Handler( );
  }

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED1_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CAMERA_RESET_GPIO_Port, CAMERA_RESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED3_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : STLINK_RX_Pin STLINK_TX_Pin */
  GPIO_InitStruct.Pin = STLINK_RX_Pin|STLINK_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CAMERA_RESET_Pin */
  GPIO_InitStruct.Pin = CAMERA_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CAMERA_RESET_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartCameraTask */
/**
* @brief Function implementing the cameraTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCameraTask */
void StartCameraTask(void *argument)
{
  /* USER CODE BEGIN StartCameraTask */

	/* Infinite loop */
	for (;;)
	{
		// Waits a direct-to-task notification to cameraTaskHandle
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

//		// Suspend DCMI
//		HAL_DCMI_Suspend(&hdcmi);
//
//    	// Invalidate the cache
//    	SCB_InvalidateDCache_by_Addr((uint32_t*)frame_buffer, FRAME_BYTES);

		// Signal processing
	    HAL_GPIO_TogglePin(GPIOB, LED1_Pin);

		/* Transmit image via UART with 100 ms timeout */
	    //for(int i=0;i<FRAME_BYTES;i++)
	    //	HAL_UART_Transmit(&huart5, &(frame_buffer[i]), 1, 100);
		//  CDC_Transmit_FS((uint8_t *)(frame_buffer[i]), 1);
	        
	    /* Transmit image via USB Vitual COM  */
	    if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED)
	    {
	    	// Invalidate the cache
	    	SCB_InvalidateDCache_by_Addr((uint32_t*)frame_buffer, FRAME_BYTES);

            /*
             * Synthetic image test.
             * Do not use camera data.
             */
	    	//FillGrayRamp_Y8((uint8_t *)frame_buffer, WIDTH, HEIGHT);
            // Or:
            // FillGrayRamp_Y8((uint8_t *)frame_buffer, WIDTH, HEIGHT);

            /*
             * Since CPU wrote the buffer, clean cache.
             * Do NOT invalidate here.
             */
            //SCB_CleanDCache_by_Addr((uint32_t *)frame_buffer, FRAME_BYTES);

			//PSRAM_Write(0,frame_buffer,FRAME_BYTES);
			//PSRAM_Read(0,frame_buffer,FRAME_BYTES);

	    	USB_SendFrame((uint8_t *)frame_buffer, FRAME_BYTES);
	    	//USB_SendBuffer((uint8_t *)frame_buffer, FRAME_BYTES);
	    }
        else
        {
    		// Resume DCMI
            HAL_DCMI_Resume(&hdcmi);
        }

		osDelay(1);
	}
  /* USER CODE END StartCameraTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
