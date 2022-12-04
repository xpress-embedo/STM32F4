#include "tft.h"
#include <stdint.h>
#include "ili9341.h"
#include "lvgl/lvgl.h"

// Contains the callback functions to interact with the display and manipulate
// low level drawing behavior
static lv_disp_drv_t disp_drv;

// Private Functions
static void TFT_FlushSlow(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_p);
static void TFT_Flush(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_p);
static void Monitor_CB(lv_disp_drv_t * d, uint32_t t, uint32_t p);
static void Display_Init( void );

void TFT_Init( void )
{
  // Draw Buffer (Second Buffer is Optional)
  // These are simple arrays used by LVGL to render the screen content.
  // Once the rendering is ready the content of the draw buffer is sent to the display
  // using the flush_cb callback function, which is set in display driver
  static lv_color_t disp_buf1[TFT_HOR_RES*10];
  static lv_color_t disp_buf2[TFT_HOR_RES*10];

  // Contains the Internal Graphic Buffer(s) called draw buffers
  static lv_disp_draw_buf_t buf;

  // Initialize the display buffer with the buffers
  lv_disp_draw_buf_init( &buf, disp_buf1, disp_buf2, TFT_HOR_RES*10u );

  lv_disp_drv_init( &disp_drv );

  // Initialize display
  Display_Init();

  disp_drv.draw_buf = &buf;
  disp_drv.flush_cb = TFT_Flush;
  disp_drv.monitor_cb = Monitor_CB;
  disp_drv.hor_res = TFT_HOR_RES;
  disp_drv.ver_res = TFT_VER_RES;
  disp_drv.sw_rotate = 0;           // Set to one if wanted to rotate the screen
  lv_disp_drv_register(&disp_drv);
}

uint16_t TFT_GetWidth( void )
{
  return (ILI9341_GetWidth());
}

uint16_t TFT_GetHeight( void )
{
  return (ILI9341_GetHeight());
}

// This is slow flush function
static void TFT_FlushSlow(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_p)
{
  uint16_t x, y;
  uint16_t temp2 = 0;
  lv_color_t temp;
  temp = *color_p;
  for(y = area->y1; y <= area->y2; y++)
  {
    for(x = area->x1; x <= area->x2; x++)
    {
      temp = *color_p;
      temp2 = temp.full;
      ILI9341_DrawPixel(x, y, temp2);
      color_p++;
    }
  }

  lv_disp_flush_ready(&disp_drv);
}

//Flush Function
static void TFT_Flush(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_p)
{
  #define SPI_DATA_TRANSFER_8BIT        (0u)
  #define SPI_DATA_TRANSFER_16BIT       (1u)
  // #define SPI_DATA_TRANSFER_BITS        SPI_DATA_TRANSFER_8BIT
  #define SPI_DATA_TRANSFER_BITS        SPI_DATA_TRANSFER_16BIT
  uint16_t len = 0u;
  lv_coord_t width = 0;

  // return if area is out of screen
  if(area->x2 < 0 || area->y2 < 0 || area->x1 > (TFT_HOR_RES  - 1) || area->y1 > (TFT_VER_RES  - 1))
  {
    lv_disp_flush_ready(drv);
    return;
  }

  width = (area->x2 - area->x1) + 1;

  // Truncate the area to the screen*
  uint16_t act_x1 = area->x1 < 0 ? 0 : area->x1;
  uint16_t act_y1 = area->y1 < 0 ? 0 : area->y1;
  uint16_t act_x2 = area->x2 > TFT_HOR_RES - 1 ? TFT_HOR_RES - 1 : area->x2;
  uint16_t act_y2 = area->y2 > TFT_VER_RES - 1 ? TFT_VER_RES - 1 : area->y2;

  // calculate length of data to send over SPI (make sure to multiply by 2 if
  // sending in 8-bit SPI transfer, and also set LV_COLOR_16_SWAP to 1
  // for ex. RGB color is 0xF7BE, in 8 bit mode it will be sent as 0xBE & 0xF7
  // hence set LV_COLOR_16_SWAP = 1 to swap this
  #if (SPI_DATA_TRANSFER_BITS == SPI_DATA_TRANSFER_8BIT)
  len = (act_x2 - act_x1 + 1u)*2u;
  #elif (SPI_DATA_TRANSFER_BITS == SPI_DATA_TRANSFER_16BIT)
  len = (act_x2 - act_x1 + 1u);
  #endif

  // Set Display Area
  ILI9341_SetWindow( act_x1, act_y1, act_x2, act_y2 );

  ILI9341_SendCommand(ILI9341_GRAM, 0u, 0u );

  for( uint32_t y = act_y1; y <= act_y2; y++)
  {
    #if (SPI_DATA_TRANSFER_BITS == SPI_DATA_TRANSFER_8BIT)
    ILI9341_SendData((uint8_t*)color_p, len);
    #elif (SPI_DATA_TRANSFER_BITS == SPI_DATA_TRANSFER_16BIT)
    ILI9341_Send_16BitData( (uint16_t*)color_p, len );
    #endif
    color_p += width;
  }

  lv_disp_flush_ready(&disp_drv);
}

static volatile uint32_t t_saved = 0;
void Monitor_CB(lv_disp_drv_t * d, uint32_t t, uint32_t p )
{
  t_saved = t;
}

static void Display_Init( void )
{
  ILI9341_Init();
  ILI9341_SetOrientation( LCD_LANDSCAPE );
  // ILI9341_SetOrientation( LCD_ORIENTATION_180 );
  // ILI9341_SetOrientation( LCD_PORTRAIT );
  // Some Test Code Starts
  // ILI9341_DrawPixel( 0u, 0u, ILI9341_RED );
  // ILI9341_SetOrientation( LCD_ORIENTATION_0 );
  // ILI9341_Fill( ILI9341_RED );
  // ILI9341_SetOrientation( LCD_ORIENTATION_90 );
  // ILI9341_Fill( ILI9341_ORANGE );
  // ILI9341_SetOrientation( LCD_ORIENTATION_180 );
  // ILI9341_Fill( ILI9341_PINK );
  // ILI9341_SetOrientation( LCD_ORIENTATION_270 );
  // ILI9341_Fill( ILI9341_DARKGREEN );
  // ILI9341_Fill( ILI9341_DARKCYAN );
  // ILI9341_FillRectangle( 0u, 0u, 50u, 50u, ILI9341_WHITE );
  // ILI9341_FillRectangle( 50u, 50u, 100u, 100u, ILI9341_ORANGE );
  // ILI9341_FillRectangle( 100u, 100u, 150u, 150u, ILI9341_DARKGREEN );
  // ILI9341_DrawCircle( 20, 20, 20, ILI9341_PINK);
  // ILI9341_DrawLine( 0u, 0u, ILI9341_GetWidth(), ILI9341_GetHeight(), ILI9341_DARKGREEN);
  // ILI9341_DrawHLine( 10u, 10u, 50u, ILI9341_ORANGE );
  // ILI9341_DrawVLine( 10u, 10u, 50u, ILI9341_ORANGE );
  // Some Test Code Ends
}
