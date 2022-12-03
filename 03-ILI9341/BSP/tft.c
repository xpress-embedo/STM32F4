#include "tft.h"
#include <stdint.h>
#include "ili9341.h"
#include "lvgl/lvgl.h"

// Contains the callback functions to interact with the display and manipulate
// low level drawing behavior
static lv_disp_drv_t disp_drv;

// Private Functions
static void tft_flush(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_p);
void monitor_cb(lv_disp_drv_t * d, uint32_t t, uint32_t p);

void tft_init( void )
{
  // Draw Buffer (Second Buffer is Optional)
  // These are simple arrays used by LVGL to render the screen content.
  // Once the rendering is ready the content of the draw buffer is sent to the display
  // using the flush_cb callback function, which is set in display driver
  // #define DISP_BUFF_SIZE        ((TFT_VER_RES/10)*TFT_HOR_RES)
  // static lv_color_t disp_buf1[DISP_BUFF_SIZE];
  // static lv_color_t disp_buf2[DISP_BUFF_SIZE];
  static lv_color_t disp_buf1[10u*1024u];
  static lv_color_t disp_buf2[10u*1024u];

  // Contains the Internal Graphic Buffer(s) called draw buffers
  static lv_disp_draw_buf_t buf;

  // Initialize the display buffer with the buffers
  lv_disp_draw_buf_init( &buf, disp_buf1, disp_buf2, 5u*1024u );

  lv_disp_drv_init( &disp_drv );

  ILI9341_Init();
  ILI9341_SetOrientation( LCD_ORIENTATION_180 );
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

  disp_drv.draw_buf = &buf;
  disp_drv.flush_cb = tft_flush;
  disp_drv.monitor_cb = monitor_cb;
  disp_drv.hor_res = TFT_HOR_RES;
  disp_drv.ver_res = TFT_VER_RES;
  disp_drv.sw_rotate = 0;           // Set to one if wanted to rotate the screen
  lv_disp_drv_register(&disp_drv);
}

static void tft_flush(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_p)
{
  uint32_t len = 0u;
  lv_coord_t width = (area->x2 - area->x1) + 1;

  if(area->x2 < 0 || area->y2 < 0 || area->x1 > (TFT_HOR_RES  - 1) || area->y1 > (TFT_VER_RES  - 1))
  {
    lv_disp_flush_ready(drv);
    return;
  }

  /*Return if the area is out the screen*/
  if(area->x2 < 0) return;
  if(area->y2 < 0) return;
  if(area->x1 > TFT_HOR_RES - 1) return;
  if(area->y1 > TFT_VER_RES - 1) return;

  /*Truncate the area to the screen*/
  int32_t act_x1 = area->x1 < 0 ? 0 : area->x1;
  int32_t act_y1 = area->y1 < 0 ? 0 : area->y1;
  int32_t act_x2 = area->x2 > TFT_HOR_RES - 1 ? TFT_HOR_RES - 1 : area->x2;
  int32_t act_y2 = area->y2 > TFT_VER_RES - 1 ? TFT_VER_RES - 1 : area->y2;

  // Set Display Area
  ILI9341_SetWindow( act_x1, act_y1, act_x2, act_y2 );
  len = (act_x2 - act_x1 + 1u)*2u;
  ILI9341_SendCommand(ILI9341_GRAM, 0u, 0u );

  for( uint32_t y = act_y1; y <= act_y2; y++)
  {
    ILI9341_Send_16BitData( (uint16_t*)color_p, len );
    color_p += width;
  }
  lv_disp_flush_ready(&disp_drv);
}

static volatile uint32_t t_saved = 0;
void monitor_cb(lv_disp_drv_t * d, uint32_t t, uint32_t p )
{
  t_saved = t;
}

