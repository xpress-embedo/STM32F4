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
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include <stdio.h>
#include <string.h>
#include "SEGGER_SYSVIEW.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* Application States */
typedef enum _Application_States_e
{
  MAIN_MENU = 0,
  LED_EFFECT,
  RTC_MENU,
  RTC_TIME_CONFIG,
  RTC_DATE_CONFIG,
  RTC_REPORT
} Application_States_e;

typedef struct _Command_s
{
  uint8_t payload[8];
  uint8_t len;
} Command_s;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define DWT_CTRL            (*(volatile uint32_t*)0xE0001000u)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void Command_Processor( Command_s *s_cmd );
static int8_t Command_Extractor( Command_s *s_cmd );
static void Led_Effect( uint8_t option );
static void Led_Effect_Callback( TimerHandle_t xTimer );
static void RTC_ReportCallback( TimerHandle_t xRTC );
static void Led_Effect1( void );
static void Led_Effect2( void );
static void Led_Effect3( void );
static void RTC_ConfigureTime( RTC_TimeTypeDef *time );
static void RTC_ConfigureDate( RTC_DateTypeDef *date );
static void RTC_ShowDateTime( void );
static uint8_t Validate_RTC_Information( RTC_TimeTypeDef *time, RTC_DateTypeDef *date );
static uint8_t Convert_To_Number( uint8_t *data, uint8_t len );
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
TaskHandle_t menu_task_handle;
TaskHandle_t command_task_handle;
TaskHandle_t print_task_handle;
TaskHandle_t led_task_handle;
TaskHandle_t rtc_task_handle;

/* We need two queues one for inputs from UART and another for printing */
QueueHandle_t q_data;
QueueHandle_t q_print;

uint8_t uart_data;

Application_States_e e_state = MAIN_MENU;
/* Software Timer for Led Effects */
TimerHandle_t led_effect_timer[3];
/* Software Timer for RTC Reporting */
TimerHandle_t rtc_timer;
const char * msg_invalid = "Entered Value is Invalid\n";
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  BaseType_t status;
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
  MX_RTC_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* Enable the Cycle Counter i.e. CYCCNT */
  DWT_CTRL |= (0x01 << 0);

  /* SystemView should be configured and start before using any FreeRTOS API's */
  SEGGER_SYSVIEW_Conf();
  SEGGER_SYSVIEW_Start();

  status = xTaskCreate(Menu_TaskHandler, "Menu Task", 200, NULL, 2, &menu_task_handle);
  configASSERT(status == pdPASS);

  status = xTaskCreate(Command_TaskHandler, "CMD Task", 200, NULL, 2, &command_task_handle );
  configASSERT(status == pdPASS);

  status = xTaskCreate(Print_TaskHandler, "Print Task", 200, NULL, 2, &print_task_handle );
  configASSERT(status == pdPASS);

  status = xTaskCreate(LED_TaskHandler, "LED Task", 200, NULL, 2, &led_task_handle );
  configASSERT(status == pdPASS);

  status = xTaskCreate(RTC_TaskHandler, "RTC Task", 200, NULL, 2, &rtc_task_handle );
  configASSERT(status == pdPASS);

  /* Create Queues */
  q_data = xQueueCreate( 10, sizeof(char) );
  configASSERT(q_data != NULL);
  q_print = xQueueCreate(10, sizeof(size_t) );

  /* Configure the UART to receive 1 byte of over interrupt */
  HAL_UART_Receive_IT(&huart1, &uart_data, 1);

  /* Create Software Timers for Led Effects */
  for( uint8_t i=0; i<3; i++ )
  {
    led_effect_timer[i] = xTimerCreate( "led_timer", pdMS_TO_TICKS(500), pdTRUE,\
                                        (void*)(i+1), Led_Effect_Callback );
  }

  /* Create Software Timer for RTC reporting */
  rtc_timer = xTimerCreate( "RTC Timer", pdMS_TO_TICKS(1000), pdTRUE, NULL, \
                            RTC_ReportCallback );

  /* Start the Scheduler */
  vTaskStartScheduler();
  /* control shouldn't come here, if it is coming then there is some problem */
  /* this failure can happen due to insufficient memory in heap */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_12;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, NCS_MEMS_SPI_Pin|CSX_Pin|OTG_FS_PSO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ACP_RST_GPIO_Port, ACP_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, RDX_Pin|WRX_DCX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, LD3_Pin|LD4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : A0_Pin A1_Pin A2_Pin A3_Pin
                           A4_Pin A5_Pin SDNRAS_Pin A6_Pin
                           A7_Pin A8_Pin A9_Pin */
  GPIO_InitStruct.Pin = A0_Pin|A1_Pin|A2_Pin|A3_Pin
                          |A4_Pin|A5_Pin|SDNRAS_Pin|A6_Pin
                          |A7_Pin|A8_Pin|A9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI5_SCK_Pin SPI5_MISO_Pin SPI5_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI5_SCK_Pin|SPI5_MISO_Pin|SPI5_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI5;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : ENABLE_Pin */
  GPIO_InitStruct.Pin = ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
  HAL_GPIO_Init(ENABLE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SDNWE_Pin */
  GPIO_InitStruct.Pin = SDNWE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(SDNWE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : NCS_MEMS_SPI_Pin CSX_Pin OTG_FS_PSO_Pin */
  GPIO_InitStruct.Pin = NCS_MEMS_SPI_Pin|CSX_Pin|OTG_FS_PSO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : B1_Pin MEMS_INT1_Pin MEMS_INT2_Pin TP_INT1_Pin */
  GPIO_InitStruct.Pin = B1_Pin|MEMS_INT1_Pin|MEMS_INT2_Pin|TP_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : B5_Pin VSYNC_Pin G2_Pin R4_Pin
                           R5_Pin */
  GPIO_InitStruct.Pin = B5_Pin|VSYNC_Pin|G2_Pin|R4_Pin
                          |R5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ACP_RST_Pin */
  GPIO_InitStruct.Pin = ACP_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ACP_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OC_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : R3_Pin R6_Pin */
  GPIO_InitStruct.Pin = R3_Pin|R6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF9_LTDC;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : A10_Pin A11_Pin BA0_Pin BA1_Pin
                           SDCLK_Pin SDNCAS_Pin */
  GPIO_InitStruct.Pin = A10_Pin|A11_Pin|BA0_Pin|BA1_Pin
                          |SDCLK_Pin|SDNCAS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : D4_Pin D5_Pin D6_Pin D7_Pin
                           D8_Pin D9_Pin D10_Pin D11_Pin
                           D12_Pin NBL0_Pin NBL1_Pin */
  GPIO_InitStruct.Pin = D4_Pin|D5_Pin|D6_Pin|D7_Pin
                          |D8_Pin|D9_Pin|D10_Pin|D11_Pin
                          |D12_Pin|NBL0_Pin|NBL1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : G4_Pin G5_Pin B6_Pin B7_Pin */
  GPIO_InitStruct.Pin = G4_Pin|G5_Pin|B6_Pin|B7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_HS_ID_Pin OTG_HS_DM_Pin OTG_HS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_HS_ID_Pin|OTG_HS_DM_Pin|OTG_HS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF12_OTG_HS_FS;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_HS_Pin */
  GPIO_InitStruct.Pin = VBUS_HS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_HS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : D13_Pin D14_Pin D15_Pin D0_Pin
                           D1_Pin D2_Pin D3_Pin */
  GPIO_InitStruct.Pin = D13_Pin|D14_Pin|D15_Pin|D0_Pin
                          |D1_Pin|D2_Pin|D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : TE_Pin */
  GPIO_InitStruct.Pin = TE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RDX_Pin WRX_DCX_Pin */
  GPIO_InitStruct.Pin = RDX_Pin|WRX_DCX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : R7_Pin DOTCLK_Pin B3_Pin */
  GPIO_InitStruct.Pin = R7_Pin|DOTCLK_Pin|B3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : HSYNC_Pin G6_Pin R2_Pin */
  GPIO_InitStruct.Pin = HSYNC_Pin|G6_Pin|R2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : I2C3_SDA_Pin */
  GPIO_InitStruct.Pin = I2C3_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
  HAL_GPIO_Init(I2C3_SDA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2C3_SCL_Pin */
  GPIO_InitStruct.Pin = I2C3_SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
  HAL_GPIO_Init(I2C3_SCL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : G7_Pin B2_Pin */
  GPIO_InitStruct.Pin = G7_Pin|B2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : G3_Pin B4_Pin */
  GPIO_InitStruct.Pin = G3_Pin|B4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF9_LTDC;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD4_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : SDCKE1_Pin SDNE1_Pin */
  GPIO_InitStruct.Pin = SDCKE1_Pin|SDNE1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback( UART_HandleTypeDef *huart )
{
  uint8_t dummy;
  /* check if Queue is full or not */
  if( xQueueIsQueueFullFromISR(q_data) != pdPASS )
  {
    /* Queue is not full, so enqueue the data in queue */
    xQueueSendFromISR( q_data, (void*)&uart_data, NULL );
  }
  else
  {
    /* if the received data is \n ? */
    if( uart_data == '\n' )
    {
      /* Make sure the last data byte of the queue is \n */
      xQueueReceiveFromISR(q_data, (void*)&dummy, NULL);    /* delete the last entry for making space for \n */
      xQueueSendFromISR( q_data, (void*)&uart_data, NULL );
    }
  }
  /* Send Notification to command handling task if the uart_data = \n */
  if( uart_data == '\n' )
  {
    xTaskNotifyFromISR( command_task_handle, 0, eNoAction, NULL);
  }
  /* Enable UART data byte reception again in IT mode */
  HAL_UART_Receive_IT(&huart1, &uart_data, 1);
}

void Menu_TaskHandler( void * parameter)
{
  uint32_t command_address;
  Command_s *s_cmd;
  int option;
  const char * msg_menu = "=======================\n"
                          "==========MENU=========\n"
                          "=======================\n"
                          "LED Effect    => 0     \n"
                          "Date and Time => 1     \n"
                          "Exit          => 2     \n"
                          "Enter your choice here:\n";
  while(1)
  {
    xQueueSend( q_print, &msg_menu, portMAX_DELAY );
    xTaskNotifyWait(0, 0, &command_address, portMAX_DELAY );
    s_cmd = (Command_s*)command_address;
    if( s_cmd->len == 1 )
    {
      option = s_cmd->payload[0] - 48;
      switch( option )
      {
        case 0:
          e_state = LED_EFFECT;
          xTaskNotify( led_task_handle, 0, eNoAction);
          break;
        case 1:
          e_state = RTC_MENU;
          xTaskNotify( rtc_task_handle, 0, eNoAction);
          break;
        case 2:
          /* implement exit */
          break;
        default:
          xQueueSend( q_print, &msg_invalid, portMAX_DELAY );
          continue;
      }
    }
    else
    {
      /* this is an invalid entry */
      xQueueSend( q_print, &msg_invalid, portMAX_DELAY );
      continue;
    }

    xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
  }
}

void Command_TaskHandler( void * parameter )
{
  BaseType_t status;
  Command_s s_command;
  while(1)
  {
    /* wait for notification for indefinite time*/
    status = xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
    if( status == pdTRUE )
    {
      /* this means that notification is reached, and hence called the process
      command function, which extracts the sent command from user */
      Command_Processor( &s_command );
    }
    /* process the user data stored in the queue */
    /* notify the command to do the relevant task */
  }
}

void Print_TaskHandler( void * parameter )
{
  uint32_t *msg;
  while(1)
  {
    xQueueReceive(q_print, &msg, portMAX_DELAY);
    HAL_UART_Transmit( &huart1, (uint8_t*)msg, strlen((char *)msg), HAL_MAX_DELAY );
  }
}

void LED_TaskHandler( void * parameter)
{
  uint32_t command_address;
  Command_s *s_cmd;
  const char * msg_led  = "=======================\n"
                          "========LED EFFECT=====\n"
                          "=======================\n"
                          "(none, e1, e2, e3)     \n"
                          "Enter your choice here:\n";
  while(1)
  {
    /* Wait for Notification */
    xTaskNotifyWait( 0, 0, NULL, portMAX_DELAY );

    /* Print Led Menu */
    xQueueSend( q_print, &msg_led, portMAX_DELAY);

    /* Wait for LED Commands (notify wait ) */
    xTaskNotifyWait(0, 0, &command_address, portMAX_DELAY );
    s_cmd = (Command_s*)command_address;

    if( s_cmd->len <= 4 )   /* none=4 and e1, e2, e3 are 2 */
    {
      if( !strcmp( (char*)s_cmd->payload, "none") )
      {
        Led_Effect(0);
      }
      else if( !strcmp( (char*)s_cmd->payload, "e1") )
      {
        Led_Effect(1);
      }
      else if( !strcmp( (char*)s_cmd->payload, "e2") )
      {
        Led_Effect(2);
      }
      else if( !strcmp( (char*)s_cmd->payload, "e3") )
      {
        Led_Effect(3);
      }
      else
      {
        /* Print Invalid Message */
        xQueueSend(q_print, &msg_invalid, portMAX_DELAY );
      }
    }
    else
    {
      /* Print Invalid Message */
      xQueueSend(q_print, &msg_invalid, portMAX_DELAY );
    }
    /* update the application state to menu state */
    e_state = MAIN_MENU;
    xTaskNotify(menu_task_handle, 0, eNoAction );
  }
}

void RTC_TaskHandler( void * parameter)
{
  #define HH_CONFIG       0u
  #define MM_CONFIG       1u
  #define SS_CONFIG       2u

  #define DATE_CONFIG     0u
  #define MONTH_CONFIG    1u
  #define YEAR_CONFIG     2u
  #define DOW_CONFIG      3u

  static uint8_t rtc_state = 0u;
  uint32_t command_address;
  Command_s *s_cmd;
  uint8_t rtc_sub_menu_option;
  RTC_TimeTypeDef time;
  RTC_DateTypeDef date;
  uint8_t temp = 0u;

  const char* msg1_rtc =  "=======================\n"
                          "==========RTC==========\n"
                          "=======================\n";

  const char* msg2_rtc  = "Configure Time   => 0  \n"
                          "Configure Date   => 1  \n"
                          "Enable Reporting => 2  \n"
                          "Exit             => 3  \n"
                          "Enter your choice ? :  \n";

  const char *msg_rtc_hh = "Enter hour(1-12):";
  const char *msg_rtc_mm = "Enter minutes(0-59):";
  const char *msg_rtc_ss = "Enter seconds(0-59):";

  const char *msg_rtc_dd  = "Enter date(1-31):";
  const char *msg_rtc_mo  = "Enter month(1-12):";
  const char *msg_rtc_dow = "Enter day(1-7 sun:1):";
  const char *msg_rtc_yr  = "Enter year(0-99):";

  const char *msg_conf = "Configuration successful\n";
  const char *msg_rtc_report = "Enable time & date reporting(y/n)?: ";

  while(1)
  {
    /* Wait till someone notifies */
    xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);

    /* Print the Menu and Show Current Date and Time Information */
    xQueueSend( q_print, &msg1_rtc, portMAX_DELAY);
    RTC_ShowDateTime();
    xQueueSend( q_print, &msg2_rtc, portMAX_DELAY);

    while( e_state != MAIN_MENU )
    {
      /* wait for command notification */
      xTaskNotifyWait(0, 0, &command_address, portMAX_DELAY );
      s_cmd = (Command_s*)command_address;

      switch( e_state )
      {
        case RTC_MENU:
          /* Process the RTC Menu Command */
          if( s_cmd->len == 1 )
          {
            rtc_sub_menu_option = s_cmd->payload[0] - 48u;
            switch( rtc_sub_menu_option )
            {
              case 0:
                e_state = RTC_TIME_CONFIG;
                xQueueSend( q_print, &msg_rtc_hh, portMAX_DELAY);
                break;
              case 1:
                e_state = RTC_DATE_CONFIG;
                xQueueSend( q_print, &msg_rtc_dd, portMAX_DELAY);
                break;
              case 2:
                e_state = RTC_REPORT;
                xQueueSend( q_print, &msg_rtc_report, portMAX_DELAY);
                break;
              case 3:
                e_state = MAIN_MENU;
                break;
              default:
                e_state = MAIN_MENU;
                xQueueSend( q_print, &msg_invalid, portMAX_DELAY);
                break;
            }
          }
          else
          {
            e_state = MAIN_MENU;
            xQueueSend( q_print, &msg_invalid, portMAX_DELAY);
          }
          break;
        case RTC_TIME_CONFIG:
          /* Get Hour (hh), minute (mm) and seconds (ss) information and  configure the RTC */
          switch ( rtc_state )
          {
            case HH_CONFIG:
              temp = Convert_To_Number(s_cmd->payload, s_cmd->len );
              time.Hours = temp;
              rtc_state = MM_CONFIG;
              /* Request the user to enter minute information */
              xQueueSend( q_print, &msg_rtc_mm, portMAX_DELAY);
              break;
            case MM_CONFIG:
              temp = Convert_To_Number(s_cmd->payload, s_cmd->len );
              time.Minutes = temp;
              rtc_state = SS_CONFIG;
              /* Request the user to enter seconds information */
              xQueueSend( q_print, &msg_rtc_ss, portMAX_DELAY);
              break;
            case SS_CONFIG:
              temp = Convert_To_Number(s_cmd->payload, s_cmd->len );
              time.Seconds = temp;
              if( Validate_RTC_Information( &time, NULL) )
              {
                /* RTC Data entered is acceptable */
                RTC_ConfigureTime(&time);
                /* Send Configuration Successful message */
                xQueueSend( q_print, &msg_conf, portMAX_DELAY );
                /* Also display updated date and time information */
                RTC_ShowDateTime();
              }
              else
              {
                /* Entered data is invalid, send the invalid message to print */
                xQueueSend( q_print, &msg_invalid, portMAX_DELAY );
              }
              rtc_state = HH_CONFIG;
              /* Reset the state */
              e_state = MAIN_MENU;
              break;
          }
          break;
        case RTC_DATE_CONFIG:
          /* Get date (dd), month (mm), year (yy) and day of week and  configure the RTC */
          switch ( rtc_state )
          {
            case DATE_CONFIG:
              temp = Convert_To_Number(s_cmd->payload, s_cmd->len );
              date.Date = temp;
              rtc_state = MONTH_CONFIG;
              /* Request the user to enter month information */
              xQueueSend( q_print, &msg_rtc_mo, portMAX_DELAY);
              break;
            case MONTH_CONFIG:
              temp = Convert_To_Number(s_cmd->payload, s_cmd->len );
              date.Month = temp;
              rtc_state = YEAR_CONFIG;
              /* Request the user to enter year information */
              xQueueSend( q_print, &msg_rtc_yr, portMAX_DELAY);
              break;
            case YEAR_CONFIG:
              temp = Convert_To_Number(s_cmd->payload, s_cmd->len );
              date.Year = temp;
              rtc_state = DOW_CONFIG;
              /* Request the user to enter day of week information */
              xQueueSend( q_print, &msg_rtc_dow, portMAX_DELAY);
              break;
            case DOW_CONFIG:
              temp = Convert_To_Number(s_cmd->payload, s_cmd->len );
              date.WeekDay = temp;
              if( Validate_RTC_Information( NULL, &date) )
              {
                /* RTC Data entered is acceptable */
                RTC_ConfigureDate(&date);
                /* Send Configuration Successful message */
                xQueueSend( q_print, &msg_conf, portMAX_DELAY );
                /* Also display updated date and time information */
                RTC_ShowDateTime();
              }
              else
              {
                /* Entered data is invalid, send the invalid message to print */
                xQueueSend( q_print, &msg_invalid, portMAX_DELAY );
              }
              rtc_state = HH_CONFIG;
              /* Reset the state */
              e_state = MAIN_MENU;
              break;
          }
          break;
        case RTC_REPORT:
          /* Enable to disable the RTC Current Time reporting over UART,
          instructor has used ITM instead of UART, but on my board SB9 bridge is
          not connected hence I have to send it over UART */
          if( s_cmd->len == 1 )
          {
            if( s_cmd->payload[0] == 'y' )
            {
              /* If timer is not running, then start the timer */
              if( xTimerIsTimerActive(rtc_timer) == pdFALSE )
              {
                xTimerStart(rtc_timer, portMAX_DELAY);
              }
            }
            else if( s_cmd->payload[0] == 'n' )
            {
              /* If timer is running, then stop it */
              if( xTimerIsTimerActive(rtc_timer) == pdTRUE )
              {
                xTimerStop(rtc_timer, portMAX_DELAY);
              }
            }
            else
            {
              xQueueSend( q_print, &msg_invalid, portMAX_DELAY );
            }
          }
          e_state = MAIN_MENU;
          break;
      }
    }
    /* Notify the Main task */
    xTaskNotify( menu_task_handle, 0, eNoAction );
  }
}

static void Command_Processor( Command_s *s_cmd )
{
  Command_Extractor(s_cmd);

  switch( e_state )
  {
    case MAIN_MENU:
      /* Notify Menu Task with the Command */
      xTaskNotify(menu_task_handle, (uint32_t)s_cmd, eSetValueWithOverwrite );
      break;
    case LED_EFFECT:
      /* Notify Led Effect Task with the Command */
      xTaskNotify(led_task_handle, (uint32_t)s_cmd, eSetValueWithOverwrite );
      break;
    case RTC_MENU:
    case RTC_TIME_CONFIG:
    case RTC_DATE_CONFIG:
    case RTC_REPORT:
      /* Notify RTC Task with the Command */
      xTaskNotify(rtc_task_handle, (uint32_t)s_cmd, eSetValueWithOverwrite );
      break;
  }
}

static int8_t Command_Extractor( Command_s *s_cmd )
{
  uint8_t item;
  uint8_t i = 0u;
  BaseType_t status;

  status = uxQueueMessagesWaiting(q_data);
  if( !status )
  {
    /* Queue is empty i.e. no data is present, hence exit from here */
    return (-1);
  }

  do
  {
    status = xQueueReceive(q_data, &item, 0);
    if( status == pdTRUE )
    {
      s_cmd->payload[i] = item;
      i++;
    }
  } while ( item != '\n' );
  s_cmd->payload[i-1] = '\0';
  s_cmd->len = i-1;
  return 0;
}

static void Led_Effect( uint8_t option )
{
  /* stop any previous effects which means stopping all previous running timers*/
  for( uint8_t idx=0; idx<3; idx++ )
  {
    xTimerStop( led_effect_timer[idx], portMAX_DELAY );
  }
  /* Turn off both leds first */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET );
  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET );

  /* Select/Start the newly selected timer */
  if( option )
  {
    /* And Start the selected led effect --> timer to produce the effect */
    xTimerStart( led_effect_timer[option-1], portMAX_DELAY );
  }
}

static void Led_Effect_Callback( TimerHandle_t xTimer )
{
  uint32_t id = (uint32_t)pvTimerGetTimerID(xTimer);
  switch( id )
  {
    case 1:
      Led_Effect1();
      break;
    case 2:
      Led_Effect2();
      break;
    case 3:
      Led_Effect3();
      break;
  }
}

static void Led_Effect1( void )
{
  HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin );
}

static void Led_Effect2( void )
{
  HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin );
}

static void Led_Effect3( void )
{
  HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin );
  HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin );
}

static void RTC_ConfigureTime( RTC_TimeTypeDef *time )
{
  time->TimeFormat = RTC_HOURFORMAT12_AM;
  time->DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  time->StoreOperation = RTC_STOREOPERATION_RESET;

  HAL_RTC_SetTime(&hrtc, time, RTC_FORMAT_BIN);
}

static void RTC_ConfigureDate( RTC_DateTypeDef *date )
{
  HAL_RTC_SetDate(&hrtc, date, RTC_FORMAT_BIN);
}

static void RTC_ShowDateTime( void )
{
  static char show_time[40];
  static char show_date[40];
  static char *time = show_time;
  static char *date = show_date;
  RTC_DateTypeDef rtc_date;
  RTC_TimeTypeDef rtc_time;
  char *format;

  memset(&rtc_date, 0, sizeof(rtc_date) );
  memset(&rtc_time, 0, sizeof(rtc_time) );

  /* Get the RTC Current Time */
  HAL_RTC_GetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN );
  /* Get the RTC Current Date */
  HAL_RTC_GetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN );

  format = (rtc_time.TimeFormat == RTC_HOURFORMAT12_AM) ? "AM" : "PM";

  /* Display time format hh:mm:ss [AM/PM] */
  sprintf((char*)show_time,"%s:\t%02d:%02d:%02d [%s]","\nCurrent Time & Date",\
          rtc_time.Hours, rtc_time.Minutes, rtc_time.Seconds,format);
  xQueueSend(q_print,&time,portMAX_DELAY);

  /* Display date Format : date-month-year */
  sprintf((char*)show_date,"\t%02d-%02d-%2d\n",rtc_date.Month, rtc_date.Date, 2000 + rtc_date.Year);
  xQueueSend(q_print,&date,portMAX_DELAY);
}

static void RTC_ReportCallback( TimerHandle_t xRTC )
{
  /* Sending Data Over UART, instructor has used the ITM but in board SB9 bridge
  is not soldered and hence ITM doesn't work */
  RTC_ShowDateTime();
}

static uint8_t Validate_RTC_Information( RTC_TimeTypeDef *time, RTC_DateTypeDef *date )
{
  uint8_t status = 1;
  if( time )
  {
    if( (time->Hours > 12) || (time->Minutes > 59) || (time->Seconds > 59) )
    {
      status = 0;
    }
  }
  if( date )
  {
    if( (date->Date > 31) || (date->WeekDay > 7) || (date->Year > 99) || (date->Month > 12) )
    {
      status = 0;
    }
  }
  return status;
}

static uint8_t Convert_To_Number( uint8_t *data, uint8_t len )
{
  uint8_t value = 0u;
  if( len > 1 )
  {
    value = ((data[0]-48)*10u) + (data[1]-48);
  }
  else
  {
    value = data[0] - 48;
  }
  return value;
}
/* USER CODE END 4 */

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
  if (htim->Instance == TIM6) {
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
