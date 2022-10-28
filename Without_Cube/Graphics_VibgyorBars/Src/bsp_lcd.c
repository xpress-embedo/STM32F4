/*
 * bsp_lcd.c
 *
 *  Created on: 26-Oct-2022
 *      Author: xpress_embedo
 */

#include "bsp_lcd.h"
#include "ili9341_reg.h"

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

GPIO_TypeDef *ltdc_io_ports[] =
{
  LCD_DATA_R2_PORT,
  LCD_DATA_R3_PORT,
  LCD_DATA_R4_PORT,
  LCD_DATA_R5_PORT,
  LCD_DATA_R6_PORT,
  LCD_DATA_R7_PORT,

  LCD_DATA_G2_PORT,
  LCD_DATA_G3_PORT,
  LCD_DATA_G4_PORT,
  LCD_DATA_G5_PORT,
  LCD_DATA_G6_PORT,
  LCD_DATA_G7_PORT,

  LCD_DATA_B2_PORT,
  LCD_DATA_B3_PORT,
  LCD_DATA_B4_PORT,
  LCD_DATA_B5_PORT,
  LCD_DATA_B6_PORT,
  LCD_DATA_B7_PORT
};

const uint8_t ltdc_pins[] =
{
  LCD_DATA_R2_PIN,
  LCD_DATA_R3_PIN,
  LCD_DATA_R4_PIN,
  LCD_DATA_R5_PIN,
  LCD_DATA_R6_PIN,
  LCD_DATA_R7_PIN,

  LCD_DATA_G2_PIN,
  LCD_DATA_G3_PIN,
  LCD_DATA_G4_PIN,
  LCD_DATA_G5_PIN,
  LCD_DATA_G6_PIN,
  LCD_DATA_G7_PIN,

  LCD_DATA_B2_PIN,
  LCD_DATA_B3_PIN,
  LCD_DATA_B4_PIN,
  LCD_DATA_B5_PIN,
  LCD_DATA_B6_PIN,
  LCD_DATA_B7_PIN
};

#define LCD_TOTAL_PINS      (sizeof(ltdc_pins)/sizeof(ltdc_pins[0]))

const uint8_t total_ltdc_pins = LCD_TOTAL_PINS;

/*--------------------------- Static Functions -------------------------------*/
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
  // REG_SET_VAL( pSPI->CR1, 0x06, 0x07, SPI_CR1_BR_Pos );   // SPI Clock 90MHz/128 = 0.70MHz
  // REG_SET_VAL( pSPI->CR1, 0x7U, 0x7U, SPI_CR1_BR_Pos );   // SPI clck = 90MHz/256 ==> 0.35 MHz

  CLR_BIT( pSPI->CR1, SPI_CR1_CPOL_Pos );       // CPOL = 0
  CLR_BIT( pSPI->CR1, SPI_CR1_CPHA_Pos );       // CPHA = 0

  CLR_BIT( pSPI->CR2, SPI_CR2_FRF_Pos );        // SPI Motorola Frame Format Selected

}

static void LCD_Reset( void )
{
  LCD_RESX_LOW();
  delay();      // Small software delay
  delay();
  LCD_RESX_HIGH();
  delay();
}

static void LCD_Config( void )
{
  /* array to hold the command parameters*/
  uint8_t params[15] = {0};

  LCD_Write_Cmd(ILI9341_SWRESET);
  LCD_Write_Cmd(ILI9341_POWERB);
//  LCD_Write_Cmd( 0xAA );
//  LCD_Write_Cmd( 0x55 );
//  while(1);
  params[0] = 0x00;
  params[1] = 0xD9;
  params[2] = 0x30;
  LCD_Write_Data(params, 3);

  LCD_Write_Cmd(ILI9341_POWER_SEQ);
  params[0]= 0x64;
  params[1]= 0x03;
  params[2]= 0X12;
  params[3]= 0X81;
  LCD_Write_Data(params, 4);

  LCD_Write_Cmd(ILI9341_DTCA);
  params[0]= 0x85;
  params[1]= 0x10;
  params[2]= 0x7A;
  LCD_Write_Data(params, 3);

  LCD_Write_Cmd(ILI9341_POWERA);
  params[0]= 0x39;
  params[1]= 0x2C;
  params[2]= 0x00;
  params[3]= 0x34;
  params[4]= 0x02;
  LCD_Write_Data(params, 5);

  LCD_Write_Cmd(ILI9341_PRC);
  params[0]= 0x20;
  LCD_Write_Data(params, 1);

  LCD_Write_Cmd(ILI9341_DTCB);
  params[0]= 0x00;
  params[1]= 0x00;
  LCD_Write_Data(params, 2);

  LCD_Write_Cmd(ILI9341_POWER1);
  params[0]= 0x1B;
  LCD_Write_Data(params, 1);

  LCD_Write_Cmd(ILI9341_POWER2);
  params[0]= 0x12;
  LCD_Write_Data(params, 1);

  LCD_Write_Cmd(ILI9341_VCOM1);
  params[0]= 0x08;
  params[1]= 0x26;
  LCD_Write_Data(params, 2);

  LCD_Write_Cmd(ILI9341_VCOM2);
  params[0]= 0XB7;
  LCD_Write_Data(params, 1);


  LCD_Write_Cmd(ILI9341_PIXEL_FORMAT);
  params[0]= 0x55; //select RGB565
  LCD_Write_Data(params, 1);

  LCD_Write_Cmd(ILI9341_FRMCTR1);
  params[0]= 0x00;
  params[1]= 0x1B;//frame rate = 70
  LCD_Write_Data(params, 2);

  LCD_Write_Cmd(ILI9341_DFC);    // Display Function Control
  params[0]= 0x0A;
  params[1]= 0xA2;
  LCD_Write_Data(params, 2);

  LCD_Write_Cmd(ILI9341_3GAMMA_EN);    // 3Gamma Function Disable
  params[0]= 0x02;
  LCD_Write_Data(params, 1);

  LCD_Write_Cmd(ILI9341_GAMMA);
  params[0]= 0x01;
  LCD_Write_Data(params, 1);

  LCD_Write_Cmd(ILI9341_PGAMMA);    //Set Gamma
  params[0]= 0x0F;
  params[1]= 0x1D;
  params[2]= 0x1A;
  params[3]= 0x0A;
  params[4]= 0x0D;
  params[5]= 0x07;
  params[6]= 0x49;
  params[7]= 0X66;
  params[8]= 0x3B;
  params[9]= 0x07;
  params[10]= 0x11;
  params[11]= 0x01;
  params[12]= 0x09;
  params[13]= 0x05;
  params[14]= 0x04;
  LCD_Write_Data(params, 15);

  LCD_Write_Cmd(ILI9341_NGAMMA);
  params[0]= 0x00;
  params[1]= 0x18;
  params[2]= 0x1D;
  params[3]= 0x02;
  params[4]= 0x0F;
  params[5]= 0x04;
  params[6]= 0x36;
  params[7]= 0x13;
  params[8]= 0x4C;
  params[9]= 0x07;
  params[10]= 0x13;
  params[11]= 0x0F;
  params[12]= 0x2E;
  params[13]= 0x2F;
  params[14]= 0x05;
  LCD_Write_Data(params, 15);

  LCD_Write_Cmd(ILI9341_RGB_INTERFACE);
  params[0] = 0xC2; //Data is fetched during falling edge of DOTCLK
  LCD_Write_Data(params, 1);

  LCD_Write_Cmd(ILI9341_INTERFACE);
  params[0] = 0x00;
  params[1] = 0x00;
  params[2] = 0x06;
  LCD_Write_Data(params, 3);

  LCD_Write_Cmd(ILI9341_SLEEP_OUT); //Exit Sleep
  delay();
  delay();
  LCD_Write_Cmd(ILI9341_DISPLAY_ON); //display on
  delay();
  delay();
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

  for( idx=0; idx<length; idx++ )
  {
    LCD_CSX_LOW();
    while( !READ_BIT(pSPI->SR, SPI_SR_TXE_Pos ) );
    while( !READ_BIT(pSPI->SR, SPI_SR_TXE_Pos ) );
    SET_VALUE( pSPI->DR, buffer[idx] );
    while( READ_BIT( pSPI->SR, SPI_SR_BSY_Pos ) );
    LCD_CSX_HIGH();
  }
}

static void delay( void )
{
  uint32_t idx = 0;
  for( idx=0; idx<(0xFFFF*20u); idx++ )
  {
    // don't do any there here.
  }
}
