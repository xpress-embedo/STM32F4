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
#include "lv_conf.h"
#include "hal_stm_lvgl/tft/tft.h"
#include "hal_stm_lvgl/touchpad/touchpad.h"
#include "lvgl/lvgl.h"

#include "lvgl/examples/lv_examples.h"
#include "display_mng.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define DEBUG_BUFFER_SIZE         (50u)

#define LED_3_TASK_TIME           (1000u) /* In milliseconds */
#define LED_4_TASK_TIME           (1000u) /* In milliseconds */
#define LVGL_TASK_TIME            (5u)    /* In milliseconds */
#define DISP_MNG_TASK_TIME        (100u)  /* In milliseconds */
#define TRIG_ADC_CONV_TASK_TIME   (100u)  /* In milliseconds */
#define DEBUG_PRINT_TASK_TIME     (1000u) /* In milliseconds */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static uint8_t led_3_state = FALSE;
static uint8_t led_4_state = FALSE;
static uint32_t led_3_timestamp = 0u;
static uint32_t led_4_timestamp = 0u;
static uint32_t lvgl_timestamp = 0u;
static uint32_t disp_mng_timestamp = 0u;
static uint32_t trig_adc_conv_timestamp = 0u;
static uint32_t debug_print_timestamp = 0u;

static uint8_t adc_data;
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
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void LTDC_Clock_Init( void );
static void MX_LED_GPIO_Init(void);
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
  uint8_t idx = 0u;
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
  LTDC_Clock_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  MX_LED_GPIO_Init();
  lv_init();
  TFT_Init();
  touchpad_init();

  // rotate the display
  lv_disp_set_rotation( lv_disp_get_default(), LV_DISP_ROT_270 );

  // lv_example_label_1();
  // lv_example_btn_1();
  // lv_example_scroll_1();

  // Start ADC Conversion
  HAL_ADC_Start_IT(&hadc1);

  led_3_state = FALSE;
  led_4_state = FALSE;

  lvgl_timestamp = HAL_GetTick();
  led_3_timestamp = HAL_GetTick();
  led_4_timestamp = HAL_GetTick();
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
        // HAL_UART_Transmit(&huart1, (uint8_t*)dbg_buffer, dbg_size, 1000u);

        // Storing Temperature Data
        if( temp_sensor_idx < 10u )
        {
          // store the data in buffer
          temp_sensor[temp_sensor_idx] = adc_data;
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
              HAL_UART_Transmit(&huart1, (uint8_t*)dbg_buffer, dbg_size, 1000u);
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
        // HAL_UART_Transmit(&huart1, (uint8_t*)dbg_buffer, dbg_size, 1000u);
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
      Display_Mng();
    }

    /* Task for LVGL */
    if( HAL_GetTick() - lvgl_timestamp > LVGL_TASK_TIME )
    {
      lvgl_timestamp = HAL_GetTick();
      lv_timer_handler();
    }

    /* Task for Led 3 */
    if( HAL_GetTick() - led_3_timestamp > LED_3_TASK_TIME )
    {
      led_3_timestamp = HAL_GetTick();
      if( led_3_state )
      {
        led_3_state = FALSE;
        /* Setting Pin High will turn off the Led */
        HAL_GPIO_WritePin( LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET );
      }
      else
      {
        led_3_state = TRUE;
        /* Setting Pin Low will turn on the Led */
        HAL_GPIO_WritePin( LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET );
      }
    }
    /* Task for Led 4 */
    if( HAL_GetTick() - led_4_timestamp > LED_4_TASK_TIME )
    {
      led_4_timestamp = HAL_GetTick();
      if( led_4_state )
      {
        led_4_state = FALSE;
        /* Setting Pin High will turn off the Led */
        HAL_GPIO_WritePin( LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET );
      }
      else
      {
        led_4_state = TRUE;
        /* Setting Pin Low will turn on the Led */
        HAL_GPIO_WritePin( LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET );
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  sConfig.Channel = ADC_CHANNEL_13;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/* USER CODE BEGIN 4 */

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_LED_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, LD3_Pin|LD4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LD3_Pin LD4_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
}

uint8_t * Display_GetTempData( void )
{
  return temp_sensor_1sec;
}

/**
  * @brief  This function generated the 6.25 MHz Clock for LTDC peripheral
  *         Ideally this piece of code should be the part of the
  *         SystemClock_Config(void) function, but since we disabled the LTDC
  *         Code generation in CubeMX, this is not generated and hence a
  *         separate function is needed.
  * @retval None
  */
static void LTDC_Clock_Init( void )
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  /* Configures LTDC clock to 6.25MHz */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 50;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
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
  adc_data = HAL_ADC_GetValue(hadc);
  adc_busy = TRUE;
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
