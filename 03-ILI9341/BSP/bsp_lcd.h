#include "main.h"

// LCD Height and Width
#define BSP_LCD_WIDTH               (240u)
#define BSP_LCD_HEIGHT              (320u)

// LCD Orientation
#define PORTRAIT                    (0u)
#define LANDSCAPE                   (1u)
#define BSP_LCD_ORIENTATION         (PORTRAIT)

#if(BSP_LCD_ORIENTATION == PORTRAIT)
#define BSP_LCD_ACTIVE_WIDTH        (BSP_LCD_WIDTH)
#define BSP_LCD_ACTIVE_HEIGHT       (BSP_LCD_HEIGHT)
#elif(BSP_LCD_ORIENTATION == LANDSCAPE)
#define BSP_LCD_ACTIVE_WIDTH        (BSP_LCD_HEIGHT)
#define BSP_LCD_ACTIVE_HEIGHT       (BSP_LCD_WIDTH)
#else
#error "LCD Orientation Not Defined"
#endif

void BSP_LCD_Init( void );
void BSP_LCD_SetOrientation( uint8_t orientation );
void BSP_LCD_Write( uint16_t *buffer, uint16_t length );
void BSP_LCD_SetBackgroundColor( uint32_t rgb888 );
void BSP_LCD_FillRectangle(uint32_t rgb888, uint16_t x_start, uint16_t x_width, uint16_t y_start, uint16_t y_height);