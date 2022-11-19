#include "bsp_lcd.h"
#include "ili9341.h"

// We can't use screen sized frame buffer hence we will use a small draw buffer
// we can't use screen size buffer because of RAM usage
#define DRAW_BUFFER_SIZE                  (10u*1024u)
uint8_t bsp_draw_buffer1[DRAW_BUFFER_SIZE] = { 0u };
uint8_t bsp_draw_buffer2[DRAW_BUFFER_SIZE] = { 0u };

void BSP_LCD_Init( void )
{
  ILI9341_Init();
  ILI9341_SetWindow( 0u, 0u, BSP_LCD_ACTIVE_WIDTH, BSP_LCD_ACTIVE_HEIGHT );
  BSP_LCD_SetOrientation( BSP_LCD_ORIENTATION );
}

void BSP_LCD_SetOrientation( uint8_t orientation )
{
  ILI9341_SetOrientation(orientation);
}

void BSP_LCD_Write( uint16_t *buffer, uint32_t length )
{

}

void BSP_LCD_SetBackgroundColor( uint32_t rgb888 )
{

}

void BSP_LCD_FillRectangle(uint32_t rgb888, uint16_t x_start, uint16_t x_width, uint16_t y_start, uint16_t y_height)
{

}
