#include "ili9341.h"
#include "main.h"
#include "stdlib.h"

#define RESET_DELAY()           COVER(HAL_Delay(200);)
#define RESET_LOW()
#define RESET_HIGH()
#define CS_LOW()                COVER(HAL_GPIO_WritePin( CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET); )
#define CS_HIGH()               COVER(HAL_GPIO_WritePin( CS_GPIO_Port, CS_Pin, GPIO_PIN_SET); )
#define DC_LOW()                COVER(HAL_GPIO_WritePin( DC_GPIO_Port, DC_Pin, GPIO_PIN_RESET); )
#define DC_HIGH()               COVER(HAL_GPIO_WritePin( DC_GPIO_Port, DC_Pin, GPIO_PIN_SET); )
#define BLK_OFF()               // Backlight Off Macro
#define BLK_ON()                // Backlight On Macro

#define ILI9341_MADCTL_MY       (0x80u)   // Bottom to top
#define ILI9341_MADCTL_MX       (0x40u)   // Right to left
#define ILI9341_MADCTL_MV       (0x20u)   // Reverse Mode
#define ILI9341_MADCTL_ML       (0x10u)   // LCD refresh Bottom to top
#define ILI9341_MADCTL_RGB      (0x00u)   // Led-Green-Blue pixel order
#define ILI9341_MADCTL_BGR      (0x08u)   // Blue-Green-Red pixel order
#define ILI9341_MADCTL_MH       (0x04u)   // LCD refresh right to left


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

static LCD_Orientation_e lcd_orientation = LCD_PORTRAIT;
static uint16_t lcd_width = ILI9341_LCD_WIDTH;
static uint16_t lcd_height = ILI9341_LCD_HEIGHT;

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

void ILI9341_Send_16BitData( uint16_t *data, uint16_t length )
{
  uint16_t idx = 0u;
  uint8_t value[2] = { 0 };
  CS_LOW();
  DC_HIGH();
  for( idx=0; idx<length; idx++ )
  {
    value[0] = data[idx] >> 8u;
    value[1] = data[idx] & 0xFF;
    HAL_SPI_Transmit( &hspi2, value, 2u, 10u );
  }
  CS_HIGH();
  // Earlier the plan was to use the 16-bit mode of STM32 SPI
  // but facing some problem, will work on that little later
  // HAL_SPI_DeInit( &hspi2 );
  // MX_SPI2_Init();
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

void ILI9341_SetOrientation( LCD_Orientation_e orientation )
{
  uint8_t data = 0x00;

  switch (orientation)
  {
  case LCD_ORIENTATION_0:
    data = (ILI9341_MADCTL_MY | ILI9341_MADCTL_BGR);
    lcd_width = ILI9341_LCD_WIDTH;
    lcd_height = ILI9341_LCD_HEIGHT;
    break;
  case LCD_ORIENTATION_90:
    data = (ILI9341_MADCTL_MV | ILI9341_MADCTL_BGR);
    lcd_width = ILI9341_LCD_HEIGHT;
    lcd_height = ILI9341_LCD_WIDTH;
    break;
  case LCD_ORIENTATION_180:
    data = (ILI9341_MADCTL_MX | ILI9341_MADCTL_BGR);
    lcd_width = ILI9341_LCD_WIDTH;
    lcd_height = ILI9341_LCD_HEIGHT;
    break;
  case LCD_ORIENTATION_270:
    data = (ILI9341_MADCTL_MX | ILI9341_MADCTL_MY | ILI9341_MADCTL_MV | ILI9341_MADCTL_BGR);
    lcd_width = ILI9341_LCD_HEIGHT;
    lcd_height = ILI9341_LCD_WIDTH;
    break;
  default:
    orientation = LCD_ORIENTATION_0;
    data = (ILI9341_MADCTL_MY | ILI9341_MADCTL_BGR);
    break;
  }
  lcd_orientation = orientation;
  ILI9341_SendCommand(ILI9341_MAC, &data, 1u );
}

LCD_Orientation_e ILI9341_GetOrientation( void )
{
  return lcd_orientation;
}

uint16_t ILI9341_GetWidth( void )
{
  return lcd_width;
}

uint16_t ILI9341_GetHeight( void )
{
  return lcd_height;
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

  ILI9341_SetWindow( 0u, 0u, (lcd_width-1u), (lcd_height-1u) );
  
  ILI9341_SendCommand(ILI9341_GRAM, 0u, 0u );
  while( total_pixel_counts )
  {
    ILI9341_SendData( data, 2u );
    // ILI9341_Send_16BitData(&color, 1u);
    total_pixel_counts--;
  }
}

/**
 * @brief Draws a rectangle on Glcd.
 * 
 * Draws a rectangle on Glcd by using the specified parameters.
 * 
 * @param x_upper_left: x coordinate of the upper left rectangle corner. 
 * @param y_upper_left: y coordinate of the upper left rectangle corner. 
 * @param x_bottom_right: x coordinate of the lower right rectangle corner. 
 * @param y_bottom_right: y coordinate of the lower right rectangle corner. 
 * @param color: color parameter
 */
void ILI9341_Rectangle( int16_t x_upper_left, int16_t y_upper_left, int16_t x_bottom_right, int16_t y_bottom_right, uint16_t color)
{
  ILI9341_DrawVLine( y_upper_left, y_bottom_right, x_upper_left, color);
  ILI9341_DrawHLine( x_upper_left, x_bottom_right, y_bottom_right, color);
  ILI9341_DrawVLine( y_upper_left, y_bottom_right, x_bottom_right, color);
  ILI9341_DrawHLine( x_upper_left, x_bottom_right, y_upper_left, color);
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

/**
 * @brief Draws a Circle on ILI9341 LCD.
 * 
 * Draws a Circle on Glcd by using the specified parameters.
 * <a href="https://en.wikipedia.org/wiki/Midpoint_circle_algorithm">
 * Mid Point Circle Algorithm Weblink</a>
 * 
 * @param x_center: x coordinate of the circle center.
 * @param y_center: y coordinate of the circle center.
 * @param radius: radius of the circle.
 * @param color: color parameter. Valid values in format RGB565
 */
void ILI9341_DrawCircle( int16_t x_center, int16_t y_center, int16_t radius, uint16_t color)
{
  int16_t x = radius;
  int16_t y = 0;
  int16_t err = 0;
  int16_t temp1, temp2;
  
  while (x >= y)
  {
    temp1 = x_center + x;
    temp2 = y_center + y;
    if( (temp1 >=0) && (temp1 < lcd_width ) && \
        (temp2 >= 0) && (temp2 < lcd_height) )
    {
      ILI9341_DrawPixel( temp1, temp2, color);
    }
    temp1 = x_center + y;
    temp2 = y_center + x;
    if( (temp1 >=0) && (temp1 < lcd_width) && \
        (temp2 >= 0) && (temp2 < lcd_height) )
    {
      ILI9341_DrawPixel(temp1, temp2, color);
    }
    
    temp1 = x_center - y;
    temp2 = y_center + x;
    if( (temp1 >=0) && (temp1 < lcd_width) && \
        (temp2 >= 0) && (temp2 < lcd_height) )
    {
      ILI9341_DrawPixel(temp1, temp2, color);
    }
    
    temp1 = x_center - x;
    temp2 = y_center + y;
    if( (temp1 >=0) && (temp1 < lcd_width) && \
        (temp2 >= 0) && (temp2 < lcd_height) )
    {
      ILI9341_DrawPixel(temp1, temp2, color);
    }
    
    temp1 = x_center - x;
    temp2 = y_center - y;
    if( (temp1 >=0) && (temp1 < lcd_width) && \
        (temp2 >= 0) && (temp2 < lcd_height) )
    {
      ILI9341_DrawPixel(temp1, temp2, color);
    }
    
    temp1 = x_center - y;
    temp2 = y_center - x;
    if( (temp1 >=0) && (temp1 < lcd_width) && \
        (temp2 >= 0) && (temp2 < lcd_height) )
    {
      ILI9341_DrawPixel(temp1, temp2, color);
    }
    
    temp1 = x_center + y;
    temp2 = y_center - x;
    if( (temp1 >=0) && (temp1 < lcd_width) && \
        (temp2 >= 0) && (temp2 < lcd_height) )
    {
      ILI9341_DrawPixel(temp1, temp2, color);
    }
    
    temp1 = x_center + x;
    temp2 = y_center - y;
    if( (temp1 >=0) && (temp1 < lcd_width) && \
        (temp2 >= 0) && (temp2 < lcd_height) )
    {
      ILI9341_DrawPixel(temp1, temp2, color);
    }
    
    y += 1;
    err += 1 + 2*y;
    if (2*(err-x) + 1 > 0)
    {
      x -= 1;
      err += 1 - 2*x;
    }
  }
}

/**
 * @brief Draws a line on LCD.
 * 
 * Draws a line on LCD by using the specified parameters.
 * <a href="https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm">
 * Algorithm Used</a>
 * <a href="http://www.edaboard.com/thread68526.html#post302856"> Program Used
 * </a>
 * 
 * @param x_start: x coordinate of start point.
 * @param y_start: x coordinate of start point.
 * @param x_end: x coordinate of end point.
 * @param y_end: y coordinate of end point.
 * @param color: color parameter.
 */
void ILI9341_DrawLine( int16_t x_start, int16_t y_start, \
                       int16_t x_end, int16_t y_end, uint16_t color )
{
  int16_t x, y, addx, addy, dx, dy;
  int32_t P;
  int16_t i;
  dx = abs((int16_t)(x_end - x_start));
  dy = abs((int16_t)(y_end - y_start));
  x = x_start;
  y = y_start;
  
  if(x_start > x_end)
    addx = -1;
  else
    addx = 1;
  
  if(y_start > y_end)
    addy = -1;
  else
    addy = 1;
  
  if(dx >= dy)
  {
    P = 2*dy - dx;
    
    for(i=0; i<=dx; ++i)
    {
      ILI9341_DrawPixel( x, y, color );
      if(P < 0)
      {
        P += 2*dy;
        x += addx;
      }
      else
      {
        P += 2*dy - 2*dx;
        x += addx;
        y += addy;
      }
    }
  }
  else
  {
    P = 2*dx - dy;
    for(i=0; i<=dy; ++i)
    {
      ILI9341_DrawPixel( x, y, color );
      
      if(P < 0)
      {
        P += 2*dx;
        y += addy;
      }
      else
      {
        P += 2*dx - 2*dy;
        x += addx;
        y += addy;
      }
    }
  }
}

void ILI9341_DrawHLine( int16_t x_start, int16_t y_start, int16_t width, uint16_t color )
{
  ILI9341_DrawLine( x_start, y_start, (x_start+width-1u), y_start, color);
}

void ILI9341_DrawVLine( int16_t x_start, int16_t y_start, int16_t height, uint16_t color )
{
  ILI9341_DrawLine( x_start, y_start, x_start, (y_start+height-1u), color);
}

void ILI9341_DrawTriangle( int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color)
{
  ILI9341_DrawLine(x0, y0, x1, y1, color);
  ILI9341_DrawLine(x1, y1, x2, y2, color);
  ILI9341_DrawLine(x2, y2, x0, y0, color);
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
