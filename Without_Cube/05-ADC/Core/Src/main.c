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
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum _ADC_Index_e
{
  ADC_IDX_0 = 0,
  ADC_IDX_1,
  ADC_IDX_2,
  ADC_IDX_MAX,
} ADC_Index_e;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define DEBUG_BUFFER_SIZE             (50u)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint16_t dbg_size = 0u;
char dbg_buffer[DEBUG_BUFFER_SIZE] = { 0 };
static uint8_t adc_data[ADC_IDX_MAX] = { 0x00 };
static uint8_t adc_data_idx = ADC_IDX_0;
static uint8_t adc_busy = 0x00;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_IT(&hadc1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // This is Interrupt start code ... start
    if( adc_busy )
    {
      memset( dbg_buffer, 0x00, DEBUG_BUFFER_SIZE );
      dbg_size = snprintf(dbg_buffer, DEBUG_BUFFER_SIZE, "Red = %d, Green = %d, Blue = %d \r\n", adc_data[0], adc_data[1], adc_data[2] );
      HAL_UART_Transmit(&huart2, (uint8_t*)dbg_buffer, dbg_size, 1000u);
      adc_busy = 0x00;
      HAL_Delay(500);
      HAL_ADC_Start_IT(&hadc1);
    }
    // Above is the Interrupt start code .... end

    /* Below is the ADC Example with polling method
    uint8_t red_adc = 0;
    uint8_t green_adc = 0;
    uint8_t blue_adc = 0;
    ADC_ChannelConfTypeDef adc_config = { 0 };
    memset( dbg_buffer, 0x00, DEBUG_BUFFER_SIZE );

    // Configure ADC for Red Slider
    adc_config.Channel = ADC_CHANNEL_0;
    adc_config.Rank = 1;
    adc_config.SamplingTime = ADC_SAMPLETIME_28CYCLES;
    if( HAL_ADC_ConfigChannel( &hadc1, &adc_config ) == HAL_OK )
    {
      HAL_ADC_Start(&hadc1);
      if( HAL_ADC_PollForConversion( &hadc1, 5u) == HAL_OK )
      {
        red_adc = (uint8_t)HAL_ADC_GetValue(&hadc1);
      }
    }

    // Configure ADC for Green Slider
    adc_config.Channel = ADC_CHANNEL_1;
    if( HAL_ADC_ConfigChannel( &hadc1, &adc_config ) == HAL_OK )
    {
      HAL_ADC_Start(&hadc1);
      if( HAL_ADC_PollForConversion( &hadc1, 5u) == HAL_OK )
      {
        green_adc = (uint8_t)HAL_ADC_GetValue(&hadc1);
      }
    }

    // Configure ADC for Blue Slider
    adc_config.Channel = ADC_CHANNEL_7;
    if( HAL_ADC_ConfigChannel( &hadc1, &adc_config ) == HAL_OK )
    {
      HAL_ADC_Start(&hadc1);
      if( HAL_ADC_PollForConversion( &hadc1, 5u) == HAL_OK )
      {
        blue_adc = (uint8_t)HAL_ADC_GetValue(&hadc1);
      }
    }

    dbg_size = snprintf(dbg_buffer, DEBUG_BUFFER_SIZE, "Red = %d, Green = %d, Blue = %d \r\n", red_adc, green_adc, blue_adc );
    HAL_UART_Transmit(&huart2, (uint8_t*)dbg_buffer, dbg_size, 1000u);

    HAL_Delay(1000);
    */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
/**
  * @brief  Regular conversion complete callback in non blocking mode
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  ADC_ChannelConfTypeDef adc_config = { 0 };
  // if( hadc == &hadc1)

  // Save the ADC data in the array
  if( adc_data_idx < ADC_IDX_MAX )
  {
    adc_data[adc_data_idx] = HAL_ADC_GetValue(hadc);
  }

  // Reconfigure the channel
  switch( adc_data_idx )
  {
    case ADC_IDX_0:
      // if 0 is configured then configure 1 next
      adc_config.Channel = ADC_CHANNEL_1;
      adc_data_idx = ADC_IDX_1;
      break;
    case ADC_IDX_1:
      adc_config.Channel = ADC_CHANNEL_7;
      adc_data_idx = ADC_IDX_2;
      break;
    case ADC_IDX_2:
      adc_config.Channel = ADC_CHANNEL_0;
      adc_data_idx = ADC_IDX_0;
      break;
    default:
      break;
  };

  adc_config.Rank = 1;
  adc_config.SamplingTime = ADC_SAMPLETIME_28CYCLES;

  HAL_ADC_ConfigChannel(&hadc1, &adc_config);
  // this means that one cycle is completed
  if( adc_data_idx == ADC_IDX_0 )
  {
    // one cycle is complete and next triggering will be done from the main loop
    adc_busy = 0x01;
  }
  // if one cycle is not completed, then continue triggering the ADC conversion
  else
  {
    // triggering will be done in while loop
    HAL_ADC_Start_IT( hadc );
  }
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
