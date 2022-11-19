
#include "ili9341.h"

#define COVER(x)                do {      \
                                      x;  \
                                } while(0)

#define RESET_DELAY()           COVER(HAL_Delay(100);)
#define RESET_LOW()             COVER(HAL_GPIO_WritePin( RESET_GPIO_Port, RESET_Pin, GPIO_PIN_RESET);)
#define RESET_HIGH()            COVER(HAL_GPIO_WritePin( RESET_GPIO_Port, RESET_Pin, GPIO_PIN_SET);)
#define CS_LOW()                COVER(HAL_GPIO_WritePin( CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET); )
#define CS_HIGH()               COVER(HAL_GPIO_WritePin( CS_GPIO_Port, CS_Pin, GPIO_PIN_SET); )
#define DC_LOW()                COVER(HAL_GPIO_WritePin( DC_GPIO_Port, CS_Pin, GPIO_PIN_RESET); )
#define DC_HIGH()               COVER(HAL_GPIO_WritePin( DC_GPIO_Port, CS_Pin, GPIO_PIN_SET); )
#define BLK_OFF()               // Backlight Off Macro
#define BLK_ON()                // Backlight On Macro

extern SPI_HandleTypeDef hspi2;

/*-----------------------------Private Functions------------------------------*/
static void ILI9341_Reset( void );
static void ILI9341_SoftReset( void );
static void ILI9341_SendCommand( uint8_t command );
static void ILI9341_SendData( uint8_t *data, uint16_t length );

void ILI9341_Init( void )
{
  uint8_t params[15] = { 0 };
  ILI9341_Reset();
  ILI9341_SoftReset();
  // Power Control-A
  ILI9341_SendCommand(ILI9341_POWERA);
  params[0]= 0x39;
  params[1]= 0x2C;
  params[2]= 0x00;
  params[3]= 0x34;
  params[4]= 0x02;
  ILI9341_SendData(params, 5u);

  // Power Control-B
  ILI9341_SendCommand(ILI9341_POWERB);
  params[0] = 0x00;
  params[1] = 0xD9;
  params[2] = 0x30;
  ILI9341_SendData(params, 3u);

  // Driver Timing Control A
  ILI9341_SendCommand(ILI9341_DTCA);
  params[0]= 0x85;
  params[1]= 0x10;
  params[2]= 0x7A;
  ILI9341_SendData(params, 3u);

  // Driver Timing Control B
  ILI9341_SendCommand(ILI9341_DTCB);
  params[0]= 0x00;
  params[1]= 0x00;
  ILI9341_SendData(params, 2);

  // Power of Sequence Control
  ILI9341_SendCommand(ILI9341_POWER_SEQ);
  params[0]= 0x64;
  params[1]= 0x03;
  params[2]= 0X12;
  params[3]= 0X81;
  ILI9341_SendData(params, 4u);

  // Pump Ratio Control
  ILI9341_SendCommand(ILI9341_PRC);
  params[0]= 0x20;
  ILI9341_SendData(params, 1);

  // Power Control-1
  ILI9341_SendCommand(ILI9341_POWER1);
  params[0]= 0x1B;
  ILI9341_SendData(params, 1);

  // Power Control-2
  ILI9341_SendCommand(ILI9341_POWER2);
  params[0]= 0x12;
  ILI9341_SendData(params, 1);

  // VCOM Control-1
  ILI9341_SendCommand(ILI9341_VCOM1);
  params[0]= 0x08;
  params[1]= 0x26;
  ILI9341_SendData(params, 2);

  // VCOM Control-2
  ILI9341_SendCommand(ILI9341_VCOM2);
  params[0]= 0XB7;
  ILI9341_SendData(params, 1);

  // Pixel Format Set
  ILI9341_SendCommand(ILI9341_PIXEL_FORMAT);
  params[0]= 0x55;          // 16-bit i.e. RGB565
  ILI9341_SendData(params, 1);

  // Frame Control
  ILI9341_SendCommand(ILI9341_FRMCTR1);
  params[0]= 0x00;
  params[1]= 0x1B;  //frame rate = 70 TODO: XS
  ILI9341_SendData(params, 2);

  // Display Function Control
  ILI9341_SendCommand(ILI9341_DFC);
  params[0]= 0x0A;
  params[1]= 0xA2;
  ILI9341_SendData(params, 2);

  // 3 Gamma Function Disable
  ILI9341_SendCommand(ILI9341_3GAMMA_EN);
  params[0]= 0x02;    // TODO: XS
  ILI9341_SendData(params, 1);

  // Gamma Curve Selected
  ILI9341_SendCommand(ILI9341_GAMMA);
  params[0]= 0x01;
  ILI9341_SendData(params, 1);

  // Positive Gamma Correction
  ILI9341_SendCommand(ILI9341_PGAMMA);
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
  ILI9341_SendData(params, 15);

  // Negative Gamma Correction
  ILI9341_SendCommand(ILI9341_NGAMMA);
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
  ILI9341_SendData(params, 15);

  // Exit Sleep
  ILI9341_SendCommand(ILI9341_SLEEP_OUT); //Exit Sleep
  RESET_DELAY();
  RESET_DELAY();

  // Turn on Display
  ILI9341_SendCommand(ILI9341_DISPLAY_ON);

  // LCD Direction Set
}

void ILI9341_SetWindow( uint16_t start_x, uint16_t start_y, uint16_t end_x, uint16_t end_y )
{

}

void ILI9341_WritePixel( uint16_t x, uint16_t y, uint16_t color )
{

}


/*----------------------Private Functions Definitions-------------------------*/
static void ILI9341_Reset( void )
{
  RESET_LOW();
  RESET_DELAY();
  RESET_HIGH();
  RESET_DELAY();
  CS_LOW();
  BLK_ON();
}

/** ILI9341_SoftReset
 * @brief When the Software Reset command is written, it causes a software reset
 *        It resets the commands and parameters to their S/W Reset default values
 */
static void ILI9341_SoftReset( void )
{
  uint8_t cmd = ILI9341_SWRESET;     // Software Reset Command
  ILI9341_SendCommand( cmd );
  // As per datasheet it is necessary to wait for 5 milliseconds before sending
  // the next command because display has to load all the supplier factory test
  // default values and that takes time.
  HAL_Delay(5);
}

static void ILI9341_SendCommand( uint8_t command )
{
  uint32_t timeout = 2u;
  CS_LOW();
  DC_LOW();
  HAL_SPI_Transmit( &hspi2, &command, 1u, timeout );
  CS_HIGH();
}

static void ILI9341_SendData( uint8_t *data, uint16_t length )
{
  uint32_t timeout = length*2u;     // Considering timeout as 2 * length milliseconds
  CS_LOW();
  DC_HIGH();
  HAL_SPI_Transmit( &hspi2, data, length, timeout );
  DC_LOW();
  CS_HIGH();
}
