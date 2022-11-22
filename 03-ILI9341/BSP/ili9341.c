#include "ili9341.h"
#include "main.h"

#define RESET_DELAY()           COVER(HAL_Delay(200);)
#define RESET_LOW()
#define RESET_HIGH()
#define CS_LOW()                COVER(HAL_GPIO_WritePin( CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET); )
#define CS_HIGH()               COVER(HAL_GPIO_WritePin( CS_GPIO_Port, CS_Pin, GPIO_PIN_SET); )
#define DC_LOW()                COVER(HAL_GPIO_WritePin( DC_GPIO_Port, DC_Pin, GPIO_PIN_RESET); )
#define DC_HIGH()               COVER(HAL_GPIO_WritePin( DC_GPIO_Port, DC_Pin, GPIO_PIN_SET); )
#define BLK_OFF()               // Backlight Off Macro
#define BLK_ON()                // Backlight On Macro

#define MADCTL_MY               (0x80u)   // Bottom to top
#define MADCTL_MX               (0x40u)   // Right to left
#define MADCTL_MV               (0x20u)   // Reverse Mode
#define MADCTL_ML               (0x10u)   // LCD refresh Bottom to top
#define MADCTL_RGB              (0x00u)   // Led-Green-Blue pixel order
#define MADCTL_BGR              (0x08u)   // Blue-Green-Red pixel order
#define MADCTL_MH               (0x04u)   // LCD refresh right to left


extern SPI_HandleTypeDef hspi2;

static uint8_t ILI9341_InitCommands[] = 
{
  0xEF,                 3, 0x03, 0x80, 0x02,        // This is an un-documented command https://forums.adafruit.com/viewtopic.php?f=47&t=63229&p=320378&hilit=0xef+ili9341#p320378
  ILI9341_POWERB,       3, 0x00, 0xC1, 0x30,
  ILI9341_POWER_SEQ,    4, 0x64, 0x03, 0x12, 0x81,
  ILI9341_DTCA,         3, 0x85, 0x00, 0x78,
  ILI9341_POWERA,       5, 0x39, 0x2C, 0x00, 0x34, 0x02,
  ILI9341_PRC,          1, 0x20,
  ILI9341_DTCB,         2, 0x00, 0x00,
  ILI9341_POWER1,       1, 0x23,             // Power control VRH[5:0]
  ILI9341_POWER2,       1, 0x10,             // Power control SAP[2:0];BT[3:0]
  ILI9341_VCOM1,        2, 0x3e, 0x28,       // VCM control
  ILI9341_VCOM2,        1, 0x86,             // VCM control2
  ILI9341_MAC,          1, 0x48,             // Memory Access Control
  ILI9341_VSCRSADD,     1, 0x00,             // Vertical scroll zero
  ILI9341_PIXEL_FORMAT, 1, 0x55,             // RGB565 Pixel Format Selected
  ILI9341_FRMCTR1,      2, 0x00, 0x18,
  ILI9341_DFC,          3, 0x08, 0x82, 0x27, // Display Function Control
  ILI9341_3GAMMA_EN,    1, 0x00,             // 3Gamma Function Disable
  ILI9341_GAMMA,        1, 0x01,             // Gamma curve selected
  ILI9341_PGAMMA,       15, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08,   // Set Gamma
                            0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00,
  ILI9341_NGAMMA,       15, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, // Set Gamma
                            0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F,
  ILI9341_SLEEP_OUT,    0x80,                // Exit Sleep
  ILI9341_DISPLAY_ON  , 0x80,                // Display on
  0x00                                       // End of list
};

static uint8_t tft_buffer[ILI9341_LCD_WIDTH][ILI9341_LCD_HEIGHT] = { 0 };

/*-----------------------------Private Functions------------------------------*/
static void ILI9341_Reset( void );

void ILI9341_Init( void )
{
  uint8_t *addr = ILI9341_InitCommands;
  uint8_t command = 0x00;
  uint8_t temp = 0x00;
  uint8_t num_of_args = 0x00;
  
  ILI9341_Reset();

  // Software Reset
  ILI9341_SendCommand( ILI9341_SWRESET, 0x00, 0x00 );
  HAL_Delay(150);

  do
  {
    // Get the command
    command = *addr;
    if( command )
    {
      addr++;
      // Read Length
      temp = *addr;
      num_of_args = (temp & 0x7F);
      // Point to command arguments
      addr++;
      ILI9341_SendCommand( command, addr, num_of_args );
      // now point to next command
      addr = addr + num_of_args;
      // Check if these are Sleep Out or Display On Commands
      if( temp & 0x80 )
      {
        // give some delay
        HAL_Delay(150);
      }
    }
  } while (command > 0x00 );
}

// void ILI9341_SetOrientation( uint8_t orientation )
// {
//   uint8_t param = 0u;
//   // Assuming 0=Portrait and 1=Lanscape
//   if( orientation == 0u)
//   {
//     /* Memory Access Control <portrait setting> */
//     param = MADCTL_MY | MADCTL_MX | MADCTL_BGR;
//   }
//   else if( orientation == 1u )
//   {
//     /*Memory Access Control <Landscape setting>*/
//     param = MADCTL_MV | MADCTL_MY | MADCTL_BGR;
//   }
//   // Memory Access Control command
//   // ILI9341_SendCommand(ILI9341_MAC);
//   ILI9341_SendData(&param, 1u);
// }


void ILI9341_SendCommand( uint8_t command, uint8_t *data, uint32_t length )
{
  uint32_t timeout = length*10u;                    // Considering timeout as 10 * length milliseconds
  CS_LOW();
  DC_LOW();                                         // Command Mode
  HAL_SPI_Transmit( &hspi2, &command, 1u, 10u );    // send command byte
  DC_HIGH();                                        // Data Mode
  if( length )
  {
    HAL_SPI_Transmit( &hspi2, data, length, timeout );
  }
  CS_HIGH();
}

void ILI9341_SendData( uint8_t *data, uint32_t length )
{
  uint32_t timeout = length*10u;     // Considering timeout as 10 * length milliseconds
  CS_LOW();
  DC_HIGH();
  HAL_SPI_Transmit( &hspi2, data, length, timeout );
  CS_HIGH();
}

// Set the display area
void ILI9341_SetWindow( uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end )
{
  uint8_t params[4] = { 0 };
  
  // Column Address Set (2A)
  params[0] = x_start >> 8u;
  params[1] = 0xFF & x_start;
  params[2] = x_end >> 8u;
  params[3] = 0xFF & x_end;
  ILI9341_SendCommand( ILI9341_CASET, params, 4u );

  // Row Address Set (2B) also called as page address set
  params[0] = y_start >> 8u;
  params[1] = 0xFF & y_start;
  params[2] = y_end >> 8u;
  params[3] = 0xFF & y_end;
  ILI9341_SendCommand( ILI9341_RASET, params, 4u );
}

void ILI9341_DrawPixel( uint16_t x, uint16_t y, uint16_t color )
{
  uint8_t data[2] = { (color>>8u), (color & 0xFF) };
  ILI9341_SetWindow( x, y, x, y);
  // ILI9341_SendCommand(ILI9341_GRAM, 0u, 0u );
  // ILI9341_SendData( data, 2u);
  // OR
  ILI9341_SendCommand(ILI9341_GRAM, data, 2u );
}

void ILI9341_Fill( uint16_t color )
{
  uint32_t total_pixel_counts = ILI9341_PIXEL_COUNT;    // total pixels on this TFT
  uint8_t data[2] = { (color >> 8u), (color & 0xFF) };
  
  ILI9341_SetWindow( 0u, 0u, (ILI9341_LCD_WIDTH-1u), (ILI9341_LCD_HEIGHT-1u) );
  ILI9341_SendCommand(ILI9341_GRAM, 0u, 0u );
  while( total_pixel_counts )
  {
    ILI9341_SendData( data, 2u );
    total_pixel_counts--;
  }
}

void ILI9341_FillRectangle( int16_t x_start, int16_t y_start, int16_t x_end, int16_t y_end, uint16_t color )
{
  uint32_t total_pixels_to_write = 0u;
  uint8_t data[2] = { (color >> 8u), (color & 0xFF) };

  total_pixels_to_write = ((x_end+1u)-x_start) * ((y_end+1u)-y_start);
  if( total_pixels_to_write > ILI9341_PIXEL_COUNT )
  {
    total_pixels_to_write = ILI9341_PIXEL_COUNT;
  }

  ILI9341_SetWindow( x_start, y_start, x_end, y_end );
  ILI9341_SendCommand(ILI9341_GRAM, 0u, 0u );

  while( total_pixels_to_write )
  {
    ILI9341_SendData( data, 2u );
    total_pixels_to_write--;
  }
}


/*----------------------Private Functions Definitions-------------------------*/

static void ILI9341_Reset( void )
{
  // I don't understand when ever I am using this function, my SPI stops working
  // If Reset line is connected, maybe Proteus issue, because doesn't make sense
  // that this issue will appear on the actual hardware
  RESET_LOW();
  RESET_DELAY();
  RESET_HIGH();
  RESET_DELAY();
  BLK_ON();
}
