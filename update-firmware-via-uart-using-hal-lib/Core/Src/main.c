/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	SECTOR_0 = 0,
	SECTOR_1,
	SECTOR_2,
	SECTOR_3,
	SECTOR_4,
	SECTOR_5,
	SECTOR_6,
	SECTOR_7,
} eSERTOR_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define 	FIRMWARE_SIZE	5820

#define 	SECTOR_0_BASE_ADDR		0x08000000
#define 	SECTOR_1_BASE_ADDR		0x08004000
#define 	SECTOR_2_BASE_ADDR		0x08008000
#define 	SECTOR_3_BASE_ADDR		0x0800C000
#define 	SECTOR_4_BASE_ADDR		0x08010000
#define 	SECTOR_5_BASE_ADDR		0x08020000
#define 	SECTOR_6_BASE_ADDR		0x08040000
#define 	SECTOR_7_BASE_ADDR		0x08060000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
uint8_t rx_dma_buffer[FIRMWARE_SIZE] = {0};
uint8_t receive_done;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void vectortable_move();

void flash_lock() __attribute__((section(".RamFunc")));
void flash_unlock() __attribute__((section(".RamFunc")));
void flash_erase_sector(eSERTOR_t sector) __attribute__((section(".RamFunc")));
void flash_program_byte(void* address, uint8_t* buffer, uint8_t size) __attribute__((section(".RamFunc")));
void reset_system() __attribute__((section(".RamFunc")));
void update_firmware() __attribute__((section(".RamFunc")));
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  //vectortable_move();
  HAL_UART_Receive_DMA(&huart2, rx_dma_buffer, sizeof(rx_dma_buffer));
  while (!receive_done);
  //update_firmware;
  /* USER CODE END 2 */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
/*
 *\brief
 *\param[in]
 *\param[out]
 *\retval
 */
void
vectortable_move()
{
	/* size(vector_table) = 0x194 + 0x4 - 0x00 = 0x198 */
	/* move vector table from flash to ram */
	void *volatile ram   = (void *volatile)0x20000000;
	void *volatile flash = (void *volatile)0x08000000;
	memcpy(ram, flash, 0x198);

	uint32_t *VTOR = (uint32_t *)(0xE000ED08);
	*VTOR = 0x20000000;
}

/*
 *\brief
 *\param[in]
 *\param[out]
 *\retval
 */
void
flash_lock() {
	uint32_t volatile* const FLASH_CR   = (uint32_t*)(0x40023c00 + 0x10);

	/*check LOCK bit*/
	if (((*FLASH_CR >> 31) & 1) == 0) {
		*FLASH_CR |= (1 << 31);
	}
}

/*
 *\brief
 *\param[in]
 *\param[out]
 *\retval
 */
void
flash_unlock() {
	uint32_t volatile* const FLASH_KEYR = (uint32_t*)(0x40023c00 + 0x04);
	uint32_t volatile* const FLASH_CR   = (uint32_t*)(0x40023c00 + 0x10);

	/*check LOCK bit*/
	if (((*FLASH_CR >> 31) & 1) == 1) {
		*FLASH_KEYR = 0x45670123;
		*FLASH_KEYR = 0xCDEF89AB;
	}
}

/*
 *\brief
 *\param[in]
 *\param[out]
 *\retval
 */
void
flash_erase_sector(eSERTOR_t sector) {
	uint32_t volatile* const FLASH_SR   = (uint32_t*)(0x40023c00 + 0x0C);
	uint32_t volatile* const FLASH_CR   = (uint32_t*)(0x40023c00 + 0x10);

	flash_unlock();

	/*check BUSY bit*/
	while (((*FLASH_SR >> 16) & 1) == 1) {}
	/*SET erase sector mode*/
	*FLASH_CR |= (1 << 1);
	/*select sector*/
	*FLASH_CR |= (sector << 3);
	/*start erase*/
	*FLASH_CR |= (1 << 16);
	/*check BUSY bit*/
	while (((*FLASH_SR >> 16) & 1) == 1) {}
	/*CLEAR erase sector mode*/
	*FLASH_CR &= ~(1 << 1);

	flash_lock();
}

/*
 *\brief
 *\param[in]
 *\param[out]
 *\retval
 */
void
flash_program_byte(void* address, uint8_t* buffer, uint8_t size) {
	uint32_t volatile* const FLASH_SR   = (uint32_t*)(0x40023c00 + 0x0C);
	uint32_t volatile* const FLASH_CR   = (uint32_t*)(0x40023c00 + 0x10);

	flash_unlock();

	/*check BUSY bit*/
	while (((*FLASH_SR >> 16) & 1) == 1) {}
	/*SET programming mode*/
	*FLASH_CR |= (1 << 0);
	/*write data*/
	for (uint8_t i = 0; i < size; i++) {
		*((uint8_t*)(address)++) = buffer[i];
	}
	/*CLEAR programming mode*/
	*FLASH_CR &= ~(1 << 0);
	/*check BUSY bit*/
	while (((*FLASH_SR >> 16) & 1) == 1) {}

	flash_lock();
}

/*
 *\brief
 *\param[in]
 *\param[out]
 *\retval
 */
void
reset_system() {
	uint32_t volatile* const AIRCR   = (uint32_t*)0xE000ED0C;
	*AIRCR |= (0x5FA << 16);		// register key
	*AIRCR |= (1 << 2);				// request a reset
}


/*
 *\brief
 *\param[in]
 *\param[out]
 *\retval
 */
void
update_firmware() {
	flash_erase_sector(SECTOR_0);
	flash_program_byte((void*)SECTOR_0_BASE_ADDR, rx_dma_buffer, sizeof(rx_dma_buffer));
	reset_system();
}
/* USER CODE END 4 */

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

#ifdef  USE_FULL_ASSERT
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
