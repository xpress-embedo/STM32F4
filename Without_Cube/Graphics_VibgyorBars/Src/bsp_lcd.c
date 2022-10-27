/*
 * bsp_lcd.c
 *
 *  Created on: 26-Oct-2022
 *      Author: xpress_embedo
 */

#include "project_refs.h"
#include "bsp_lcd.h"
#include "stm32f429xx.h"
#include "ili9341_reg.h"

#define GPIO_PIN_0						      (0u)
#define GPIO_PIN_1                  (1u)
#define GPIO_PIN_2                  (2u)
#define GPIO_PIN_3                  (3u)
#define GPIO_PIN_4                  (4u)
#define GPIO_PIN_5                  (5u)
#define GPIO_PIN_6                  (6u)
#define GPIO_PIN_7                  (7u)
#define GPIO_PIN_8                  (8u)
#define GPIO_PIN_9                  (9u)
#define GPIO_PIN_10                 (10u)
#define GPIO_PIN_11                 (11u)
#define GPIO_PIN_12                 (12u)
#define GPIO_PIN_13                 (13u)
#define GPIO_PIN_14                 (14u)
#define GPIO_PIN_15                 (15u)

// #ifdef STM32F429ZITx this can also be used
#ifdef STM32F429I_DISC1
// LCD Signals STM32F429 Discovery Board
#define SPI                 SPI5
#define LCD_SCL_PIN         GPIO_PIN_7
#define LCD_SCL_PORT        GPIOF
#define LCD_SDA_PIN         GPIO_PIN_9
#define LCD_SDA_PORT        GPIOF
#define LCD_RESX_PIN        GPIO_PIN_7    // Reset Pin
#define LCD_RESX_PORT       GPIOA
#define LCD_CSX_PIN         GPIO_PIN_2
#define LCD_CSX_PORT        GPIOC
#define LCD_DCX_PIN         GPIO_PIN_13
#define LCD_DCX_PORT        GPIOD

#elif STM32F407
// LCD Signals STM32F407 Board which doesn't have LCD
#define SPI                 SPI2
#define LCD_SCL_PIN         GPIO_PIN_13
#define LCD_SCL_PORT        GPIOB
#define LCD_SDI_PIN         GPIO_PIN_15
#define LCD_SDI_PORT        GPIOB
#define LCD_SDO_PIN         GPIO_PIN_2
#define LCD_SDO_PORT        GPIOC
#define LCD_RESX_PIN        GPIO_PIN_10   // Reset Pin
#define LCD_RESX_PORT       GPIOD
#define LCD_CSX_PIN         GPIO_PIN_11
#define LCD_CSX_PORT        GPIOD
#define LCD_DCX_PIN         GPIO_PIN_9
#define LCD_DCX_PORT        GPIOD
#else
#error "Supported Device is not Selected."
#endif

#define LCD_RESX_HIGH()           SET_BIT( LCD_RESX_PORT->ODR, LCD_RESX_PIN )
#define LCD_RESX_LOW()            CLR_BIT( LCD_RESX_PORT->ODR, LCD_RESX_PIN )

#define LCD_CSX_HIGH()            SET_BIT( LCD_CSX_PORT->ODR, LCD_CSX_PIN )
#define LCD_CSX_LOW()             CLR_BIT( LCD_CSX_PORT->ODR, LCD_CSX_PIN )

#define LCD_DCX_HIGH()            SET_BIT( LCD_DCX_PORT->ODR, LCD_DCX_PIN )
#define LCD_DCX_LOW()             CLR_BIT( LCD_DCX_PORT->ODR, LCD_DCX_PIN )


static void delay( void );
static void LCD_Pin_Init( void );
static void LCD_SPI_Init( void );
static void LCD_Reset( void );
static void LCD_Config( void );
static void LCD_SPI_Enable( void );
static void LCD_Write_Cmd( uint8_t cmd );
static void LCD_Write_Data( uint8_t *buffer, uint8_t length );


/**
 * @brief 
 * @param  
 */
void BSP_LCD_Init( void )
{
  LCD_Pin_Init();
  LCD_SPI_Init();
  LCD_SPI_Enable();
  LCD_Reset();
  LCD_Config();
}

static void LCD_Pin_Init( void )
{
  RCC_TypeDef *pRCC = RCC;
  GPIO_TypeDef *pGPIOA = GPIOA;
  GPIO_TypeDef *pGPIOC = GPIOC;
  GPIO_TypeDef *pGPIOD = GPIOD;
  GPIO_TypeDef *pGPIOF = GPIOF;

  // Enable the clock for GPIOA, GPIOC and GPIOD
  SET_BIT( pRCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN_Pos );
  SET_BIT( pRCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN_Pos );
  SET_BIT( pRCC->AHB1ENR, RCC_AHB1ENR_GPIODEN_Pos );
  SET_BIT( pRCC->AHB1ENR, RCC_AHB1ENR_GPIOFEN_Pos );

  // Reset Pin (This signal will reset the device)
  // REG_SET_VAL( pGPIOA->MODER, 0x01, 0x03, GPIO_MODER_MODER7_Pos );        // Output Mode is Selected
  // CLR_BIT( pGPIOA->OTYPER, GPIO_OTYPER_OT7_Pos );                         // Output Type
  // REG_SET_VAL( pGPIOA->OSPEEDR, 0x02, 0x03, GPIO_OSPEEDR_OSPEED7_Pos );   // Speed
  REG_SET_VAL( pGPIOA->MODER, 0x01, 0x03, (LCD_RESX_PIN*2u) );        // Output Mode is Selected
  CLR_BIT( pGPIOA->OTYPER, LCD_RESX_PIN );                            // Output Type
  REG_SET_VAL( pGPIOA->OSPEEDR, 0x02, 0x03, (LCD_RESX_PIN*2u) );      // Speed

  // CSX Pin (This is Chip Select, for SPI communication with the LCD)
  REG_SET_VAL( pGPIOC->MODER, 0x01, 0x03, (LCD_CSX_PIN*2u) );        // Output Mode is Selected
  CLR_BIT( pGPIOC->OTYPER, LCD_CSX_PIN );                            // Output Type
  REG_SET_VAL( pGPIOC->OSPEEDR, 0x02, 0x03, (LCD_CSX_PIN*2u) );      // Speed

  // D/CX ( 0=>Command or 1=>Parameter Selection)
  REG_SET_VAL( pGPIOD->MODER, 0x01, 0x03, (LCD_DCX_PIN*2u) );        // Output Mode is Selected
  CLR_BIT( pGPIOD->OTYPER, LCD_DCX_PIN );                            // Output Type
  REG_SET_VAL( pGPIOD->OSPEEDR, 0x02, 0x03, (LCD_DCX_PIN*2u) );      // Speed

  // Configure the GPIO pin to Alternate Function Mode
  // Configure the Alternate function mode in alternate function register
  // No Pull Up or Pull Down Settings are required for SPI Communication
  // SPI Clock (PF7)
  REG_SET_VAL( pGPIOF->MODER, 0x02, 0x03, (LCD_SCL_PIN*2u) );        // Alternate Function Mode is Selected
  CLR_BIT( pGPIOF->OTYPER, LCD_SCL_PIN );                            // Output Type
  REG_SET_VAL( pGPIOF->OSPEEDR, 0x02, 0x03, (LCD_SCL_PIN*2u) );      // Speed
  // In MODER register Alternate Function is selected, but to select which alternate
  // Function to be used we have to update the Alternate Function Register.
  // There are two register alternate function low register for 0 to 7 and then
  // alternate function high register for 8 to 15, so have to select accordingly
  REG_SET_VAL( pGPIOF->AFR[0], 0x05, 0x0F, (LCD_SCL_PIN*4u) );        // Alternate Function SPI5

  // Repeat Same for SPI SDA or MOSI (PF9)
  REG_SET_VAL( pGPIOF->MODER, 0x02, 0x03, (LCD_SDA_PIN*2u) );        // Alternate Function Mode is Selected
  CLR_BIT( pGPIOF->OTYPER, LCD_SDA_PIN );                            // Output Type
  REG_SET_VAL( pGPIOF->OSPEEDR, 0x02, 0x03, (LCD_SDA_PIN*2u) );      // Speed
  REG_SET_VAL( pGPIOF->AFR[1], 0x05, 0x0F, ( (LCD_SDA_PIN % 8u) * 4u) );

  // Set the initial state for reset, chip select and command/data pin
  SET_BIT( pGPIOC->ODR, LCD_CSX_PIN );    // Chip Select High
  SET_BIT( pGPIOA->ODR, LCD_RESX_PIN );   // Reset Pin High
  SET_BIT( pGPIOD->ODR, LCD_DCX_PIN );    // Command Data Pin as High
}

static void LCD_SPI_Enable( void )
{
  SPI_TypeDef *pSPI = SPI;
  SET_BIT( pSPI->CR1, SPI_CR1_SPE_Pos );
}

static void LCD_SPI_Init( void )
{
  // SPI should be in Half Duplex Controller mode
  // Data Format is 8-bit MSB first
  // CPOL = 0 and CPOH = 0, this is obtained from display datasheet
  // SPI Clock :
  // Chip Select is Handled by the Software
  RCC_TypeDef *pRCC = RCC;
  SPI_TypeDef *pSPI = SPI;
  // We need to enable the clock for SPI5 peripheral and as per reference manual
  // it is connected to APB2 bus
  SET_BIT( pRCC->APB2ENR, RCC_APB2ENR_SPI5EN_Pos );

  SET_BIT( pSPI->CR1, SPI_CR1_MSTR_Pos );       // Controller Mode
  SET_BIT( pSPI->CR1, SPI_CR1_BIDIMODE_Pos );   // Half-Duplex Mode
  SET_BIT( pSPI->CR1, SPI_CR1_BIDIOE_Pos );     // Transmit only (Output Enable in Bi-Directional Mode)
  CLR_BIT( pSPI->CR1, SPI_CR1_DFF_Pos );        // DFF = 8 bits
  SET_BIT( pSPI->CR1, SPI_CR1_SSM_Pos );        // Software Slave Management is enabled, no control via SPI HW module
  SET_BIT( pSPI->CR1, SPI_CR1_SSI_Pos );
  CLR_BIT( pSPI->CR1, SPI_CR1_LSBFIRST_Pos );   // MSB is First Bit, hence this bit is cleared

  // Baud Rate Control
  // We configured the HCLK as 180MHz, APB1 as 45MHz and APB2 as 90MHz
  // SPI5 is on APB2 bus, and the maximum frequency we can configure here is
  // fpclk/2 which means 90/2 = 45MHz
  // But for LCD the clock should be around 6MHz, so we have to select the
  // setting fpclk/16 = 90/16 = 5.625MHz
  REG_SET_VAL( pSPI->CR1, 0x03, 0x07, SPI_CR1_BR_Pos );   // SPI Clock 90MHz/16 = 5.625MHz

  CLR_BIT( pSPI->CR1, SPI_CR1_CPOL_Pos );       // CPOL = 0
  CLR_BIT( pSPI->CR1, SPI_CR1_CPHA_Pos );       // CPHA = 0

  CLR_BIT( pSPI->CR2, SPI_CR2_FRF_Pos );        // SPI Motorola Frame Format Selected

}

static void LCD_Reset( void )
{
  LCD_RESX_LOW();
  delay();      // Small software delay
  LCD_RESX_HIGH();
  delay();
}

static void LCD_Config( void )
{
  /* array to hold the command parameters*/
  uint8_t params[15] = {0};

  LCD_Write_Cmd(ILI9341_SWRESET);
  LCD_Write_Cmd(ILI9341_POWERB);
  params[0] = 0x00;
  params[1] = 0xD9;
  params[2] = 0x30;
  LCD_Write_Data(params, 3);
}

static void LCD_Write_Cmd( uint8_t cmd )
{
  SPI_TypeDef *pSPI = SPI;

  LCD_CSX_LOW();
  LCD_DCX_LOW();

  // TXE = 0 when buffer is empty else 1 when full, so wait here until buffer
  // is empty
  while( !READ_BIT(pSPI->SR, SPI_SR_TXE_Pos ) );
  // update the value (command) to the data register
  SET_VALUE( pSPI->DR, cmd );
  while( !READ_BIT(pSPI->SR, SPI_SR_TXE_Pos ) );

  // Since the STM32 is in controller mode, whenever the data is written in DR
  // register it will be transmitted over SDA/MOSI line

  // Make sure that the command is sent, this can be checked by the busy flag
  // in the SPI status register, 1=> means busy and 0=> means not busy
  while( READ_BIT( pSPI->SR, SPI_SR_BSY_Pos ) );

  LCD_DCX_HIGH();
  LCD_CSX_HIGH();
}

static void LCD_Write_Data( uint8_t *buffer, uint8_t length )
{
  SPI_TypeDef *pSPI = SPI;
  uint8_t idx = 0;

  LCD_CSX_LOW();
  LCD_DCX_HIGH();

  while( length-- )
  {
    while( !READ_BIT(pSPI->SR, SPI_SR_TXE_Pos ) );
    while( !READ_BIT(pSPI->SR, SPI_SR_TXE_Pos ) );
    SET_VALUE( pSPI->DR, buffer[idx] );
    while( READ_BIT( pSPI->SR, SPI_SR_BSY_Pos ) );
  }
  LCD_DCX_HIGH();
  LCD_CSX_HIGH();
}

static void delay( void )
{
  uint32_t idx = 0;
  for( idx=0; idx<(0xFFFF*20u); idx++ )
  {
    // don't do any there here.
  }
}
