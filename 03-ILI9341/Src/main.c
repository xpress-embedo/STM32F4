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
#include "tft.h"
#include "lv_conf.h"
#include "lvgl/lvgl.h"

#include "lvgl/examples/lv_examples.h"
#include "display_mng.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum _ADC_Index_e
{
  SLIDER_ADC_IDX_0 = 0,
  SLIDER_ADC_IDX_1,
  SLIDER_ADC_IDX_2,
  TEMP_ADC_IDX,
  SLIDER_ADC_IDX_MAX = TEMP_ADC_IDX,
  ADC_IDX_MAX,
} ADC_Index_e;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBUG_BUFFER_SIZE         (50u)
#define RED_SLIDER_IDX            (0u)
#define GREEN_SLIDER_IDX          (1u)
#define BLUE_SLIDER_IDX           (2u)

#define LED_1_TASK_TIME           (1000u) /* In milliseconds */
#define LED_2_TASK_TIME           (1000u) /* In milliseconds */
#define LVGL_TASK_TIME            (5u)    /* In milliseconds */
#define DISP_MNG_TASK_TIME        (100u)  /* In milliseconds */
#define TRIG_ADC_CONV_TASK_TIME   (100u)  /* In milliseconds */
#define DEBUG_PRINT_TASK_TIME     (1000u) /* In milliseconds */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static uint8_t led_1_state = FALSE;
static uint8_t led_2_state = FALSE;
static uint32_t led_1_timestamp = 0u;
static uint32_t led_2_timestamp = 0u;
static uint32_t lvgl_timestamp = 0u;
static uint32_t disp_mng_timestamp = 0u;
static uint32_t trig_adc_conv_timestamp = 0u;
static uint32_t debug_print_timestamp = 0u;

static uint8_t adc_data[ADC_IDX_MAX] = { 0x00 };
static uint8_t adc_data_idx = SLIDER_ADC_IDX_0;
static uint8_t adc_busy = FALSE;
// ADC Triggering Task Time is 100ms, this means that to get 1 second counts
// we need to store 10 samples which are 100ms apart from each other
static uint8_t temp_sensor[10u] = { 0x00 };
static uint8_t temp_sensor_idx = 0u;
// the above samples are averaged and stored in the below array
static uint8_t temp_sensor_1sec[260] = { 0 };   // 320-60
static uint16_t temp_sensor_1sec_idx = 0u;

uint16_t dbg_size = 0u;
char dbg_buffer[DEBUG_BUFFER_SIZE] = { 0 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_ADC1_Init(void);
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
  uint32_t idx = 0u;
  uint32_t temp = 0u;
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
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(10);
  lv_init();
  TFT_Init();
  // HAL_Delay(10);
  // lv_example_get_started_1();
  // lv_example_label_1();
  HAL_ADC_Start_IT(&hadc1);

  led_1_state = FALSE;
  led_2_state = FALSE;

  lvgl_timestamp = HAL_GetTick();
  led_1_timestamp = HAL_GetTick();
  led_2_timestamp = HAL_GetTick();
  trig_adc_conv_timestamp = HAL_GetTick();
  debug_print_timestamp = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* Task for Triggering ADC Conversion Start */
    if( HAL_GetTick() - trig_adc_conv_timestamp > TRIG_ADC_CONV_TASK_TIME )
    {
      trig_adc_conv_timestamp = HAL_GetTick();
      if( adc_busy == TRUE )
      {
        // memset( dbg_buffer, 0x00, DEBUG_BUFFER_SIZE );
        // dbg_size = snprintf(dbg_buffer, DEBUG_BUFFER_SIZE, "Red = %d, Green = %d, Blue = %d \r\n", adc_data[0], adc_data[1], adc_data[2] );
        // dbg_size = snprintf(dbg_buffer, DEBUG_BUFFER_SIZE, "Temp.=%d, Idx=%d, millis=%ld\r\n", adc_data[3], temp_sensor_idx, HAL_GetTick() );
        // HAL_UART_Transmit(&huart2, (uint8_t*)dbg_buffer, dbg_size, 1000u);

        if( temp_sensor_idx < 10u )
        {
          // store the data in buffer
          temp_sensor[temp_sensor_idx] = adc_data[3];
          temp_sensor_idx++;
          // this means that 1 second is elapsed and the buffer is full now
          if( temp_sensor_idx >= 10u )
          {
            // reset the counter
            temp_sensor_idx = 0u;
            // now average the samples and store in 1 second array
            temp = 0u;
            // cumulative sum
            for( idx=0; idx<10u; idx++ )
            {
              temp = temp + temp_sensor[idx];
            }
            // averaging
            temp = temp/10u;
            if( temp_sensor_1sec_idx < 260u )
            {
              // convert data to temperature and store in array
              temp = (uint8_t)((uint16_t)((uint16_t)temp * (uint16_t)330)/255);
              temp_sensor_1sec[temp_sensor_1sec_idx] = (uint8_t)(temp);
              memset( dbg_buffer, 0x00, DEBUG_BUFFER_SIZE );
              dbg_size = snprintf(dbg_buffer, DEBUG_BUFFER_SIZE, "Index = %d, Temperature = %ld\r\n", temp_sensor_1sec_idx, temp );
              HAL_UART_Transmit(&huart2, (uint8_t*)dbg_buffer, dbg_size, 1000u);
              temp_sensor_1sec_idx++;
              if( temp_sensor_1sec_idx >= 260u )
              {
                // reset the index if buffer is full
                temp_sensor_1sec_idx = 0u;
              }
            }
          }
        }

        adc_busy = FALSE;
        // Trigger Conversion Again
        HAL_ADC_Start_IT(&hadc1);
      }
      else
      {
        // memset( dbg_buffer, 0x00, DEBUG_BUFFER_SIZE );
        // dbg_size = snprintf(dbg_buffer, DEBUG_BUFFER_SIZE, "%s\r\n", "ADC Busy Shouldn't Happen" );
        // HAL_UART_Transmit(&huart2, (uint8_t*)dbg_buffer, dbg_size, 1000u);
      }
    }

    /* Task for Printing debug information */
    if( HAL_GetTick() - debug_print_timestamp > DEBUG_PRINT_TASK_TIME )
    {
      debug_print_timestamp = HAL_GetTick();
      // TODO: XS for future
    }

    /* Task Display Manager */
    if( HAL_GetTick() - disp_mng_timestamp > DISP_MNG_TASK_TIME )
    {
      disp_mng_timestamp = HAL_GetTick();
      // memset( dbg_buffer, 0x00, DEBUG_BUFFER_SIZE );
      Display_Mng();
      // dbg_size = snprintf(dbg_buffer, DEBUG_BUFFER_SIZE, "Display Mng Time=%ld\r\n", HAL_GetTick() - disp_mng_timestamp );
      // HAL_UART_Transmit(&huart2, (uint8_t*)dbg_buffer, dbg_size, 1000u);
    }

    /* Task for LVGL */
    if( HAL_GetTick() - lvgl_timestamp > LVGL_TASK_TIME )
    {
      lvgl_timestamp = HAL_GetTick();
      lv_timer_handler();
    }

    /* Task for Led 1 */
    if( HAL_GetTick() - led_1_timestamp > LED_1_TASK_TIME )
    {
      led_1_timestamp = HAL_GetTick();
      if( led_1_state )
      {
        led_1_state = FALSE;
        /* Setting Pin High will turn off the Led */
        HAL_GPIO_WritePin( LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET );
      }
      else
      {
        led_1_state = TRUE;
        /* Setting Pin Low will turn on the Led */
        HAL_GPIO_WritePin( LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET );
      }
    }
    /* Task for Led 2 */
    if( HAL_GetTick() - led_2_timestamp > LED_2_TASK_TIME )
    {
      led_2_timestamp = HAL_GetTick();
      if( led_2_state )
      {
        led_2_state = FALSE;
        /* Setting Pin High will turn off the Led */
        HAL_GPIO_WritePin( LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET );
      }
      else
      {
        led_2_state = TRUE;
        /* Setting Pin Low will turn on the Led */
        HAL_GPIO_WritePin( LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET );
      }
    }
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */
//
  /* USER CODE END SPI2_Init 2 */

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
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_GREEN_Pin|LED_RED_Pin|USER_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, DC_Pin|CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : USER_BUTTON_Pin */
  GPIO_InitStruct.Pin = USER_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_GREEN_Pin LED_RED_Pin USER_LED_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin|LED_RED_Pin|USER_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DC_Pin CS_Pin */
  GPIO_InitStruct.Pin = DC_Pin|CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_SPI_ChangeDataSizeTo16Bit( SPI_HandleTypeDef *hspi )
{
  // If in 8-bit mode, then only change to 16-bit
  if( hspi->Init.DataSize == SPI_DATASIZE_8BIT )
  {
    // first disable SPI after checking the busy flag
    while( READ_BIT(hspi->Instance->SR, SPI_SR_BSY_Msk) );
    CLEAR_BIT(hspi->Instance->CR1, SPI_CR1_SPE_Msk );
    // set the data frame register for 16-bit mode
    hspi->Init.DataSize = SPI_DATASIZE_16BIT;
    SET_BIT(hspi->Instance->CR1, SPI_CR1_DFF_Msk );
    // enable SPI again
    SET_BIT(hspi->Instance->CR1, SPI_CR1_SPE_Msk );
  }
}

void HAL_SPI_ChangeDataSizeTo8Bit( SPI_HandleTypeDef *hspi )
{
  // If in 16-bit mode, then only change to 8-bit
  if( hspi->Init.DataSize == SPI_DATASIZE_16BIT )
  {
    // first disable SPI after checking the busy flag
    while( READ_BIT(hspi->Instance->SR, SPI_SR_BSY_Msk) );
    CLEAR_BIT(hspi->Instance->CR1, SPI_CR1_SPE_Msk );
    // clear data frame register for 8-bit mode
    hspi->Init.DataSize = SPI_DATASIZE_8BIT;
    CLEAR_BIT(hspi->Instance->CR1, SPI_CR1_DFF_Msk );
    // enable SPI again
    SET_BIT(hspi->Instance->CR1, SPI_CR1_SPE_Msk );
  }
}

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
    case SLIDER_ADC_IDX_0:
      // if 0 is configured then configure 1 next
      adc_config.Channel = ADC_CHANNEL_1;
      adc_data_idx = SLIDER_ADC_IDX_1;
      break;
    case SLIDER_ADC_IDX_1:
      adc_config.Channel = ADC_CHANNEL_7;
      adc_data_idx = SLIDER_ADC_IDX_2;
      break;
    case SLIDER_ADC_IDX_2:
      adc_config.Channel = ADC_CHANNEL_8;
      adc_data_idx = TEMP_ADC_IDX;
      break;
    case TEMP_ADC_IDX:
      adc_config.Channel = ADC_CHANNEL_0;
      adc_data_idx = SLIDER_ADC_IDX_0;
    default:
      break;
  };

  adc_config.Rank = 1;
  adc_config.SamplingTime = ADC_SAMPLETIME_28CYCLES;

  HAL_ADC_ConfigChannel(&hadc1, &adc_config);
  // this means that one cycle is completed
  if( adc_data_idx == SLIDER_ADC_IDX_0 )
  {
    // one cycle is complete and next triggering will be done from the main loop
    adc_busy = TRUE;
  }
  // if one cycle is not completed, then continue triggering the ADC conversion
  else
  {
    // triggering will be done in while loop
    HAL_ADC_Start_IT( hadc );
  }
}

uint8_t Display_GetSliderCounts( uint8_t slider_type )
{
  uint8_t slider_value = 0u;
  // Here Slider ADC IDX Max is used purposely, bcz next index is of temp sensor
  if( slider_type < SLIDER_ADC_IDX_MAX )
  {
    __disable_irq();
    // Make Sure that Slider type index must match with the ADC Channel/Data Index
    slider_value = adc_data[ slider_type ];
    __enable_irq();
  }
  return slider_value;
}

uint8_t * Display_GetTempData( void )
{
  return temp_sensor_1sec;
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
