/*
 * bsp_lcd.h
 *
 *  Created on: 26-Oct-2022
 *      Author: xpress_embedo
 */

#ifndef BSP_LCD_H_
#define BSP_LCD_H_

#include "project_refs.h"
#include "board.h"
#include "stm32f429xx.h"
#include "ili9341_reg.h"

#define PORTRAIT                    (0u)
#define LANDSCAPE                   (1u)
#define BSP_LCD_ORIENTATION         (PORTRAIT)

#define BSP_LCD_HSW                 (10u)   // Horizontal Synch Width
#define BSP_LCD_HBP                 (20u)   // Horizontal Back Porch
#define BSP_LCD_HFP                 (10u)   // Horizontal Front Porch

#define BSP_LCD_VSW                 (2u)    // Vertical Synch Width
#define BSP_LCD_VBP                 (2u)    // Vertical Back Porch
#define BSP_LCD_VFP                 (4u)    // Vertical Front Porch

#define BSP_LCD_WIDTH               (240u)  // this is the real width of the lcd
#define BSP_LCD_HEIGHT              (320u)  // this is the real height of the lcd

#if (BSP_LCD_ORIENTATION == PORTRAIT )
  // in portrait mode lcd active width and height should be same as real lcd width and height
  #define BSP_LCD_ACTIVE_WIDTH      (BSP_LCD_WIDTH)
  #define BSP_LCD_ACTIVE_HEIGHT     (BSP_LCD_HEIGHT)
#elif (BSP_LCD_ORIENTATION == LANDSCAPE )
  // in landscape mode lcd active width and height are interchanged with each other
  #define BSP_LCD_ACTIVE_WIDTH      (BSP_LCD_HEIGHT)
  #define BSP_LCD_ACTIVE_HEIGHT     (BSP_LCD_WIDTH)
#else
#error "Orientation Not Selected."
#endif

// #define BSP_LTDC_LAYER_WIDTH        (BSP_LCD_ACTIVE_WIDTH)
// #define BSP_LTDC_LAYER_HEIGHT       (BSP_LCD_ACTIVE_HEIGHT)
// The image is added "STM32F4_TFT_Layout.png" in the documentation folder,
// this is the display layout, and same is displayed on the TFT screen.
#define BSP_LTDC_LAYER_WIDTH        (200u)
#define BSP_LTDC_LAYER_HEIGHT       (64u)

#define BSP_LTDC_LAYER_H_START      (20u)
#define BSP_LTDC_LAYER_H_STOP       (220u)  // BSP_LTDC_LAYER_H_START + BSP_LTDC_LAYER_WIDTH = 20+200 (these are not used)
#define BSP_LTDC_LAYER_V_START      (40u)
#define BSP_LTDC_LAYER_V_STOP       (104u)  // BSP_LTDC_LAYER_V_START +  BSP_LTDC_LAYER_HEIGHT = 104u (these are not used)

// LCD Pixel Formats
#define BSP_LCD_PIXEL_FMT_L8        (1u)
#define BSP_LCD_PIXEL_FMT_RGB565    (2u)
#define BSP_LCD_PIXEL_FMT_RGB666    (3u)
#define BSP_LCD_PIXEL_FMT_RGB888    (4u)

#define BSP_LCD_PIXEL_FMT           (BSP_LCD_PIXEL_FMT_RGB565)

#define RGB888(r,g,b)               (((r) << 16) | ((g) << 8) | (b))

#define VIOLET                      RGB888( 148u, 0u,   211u )
#define INDIGO                      RGB888( 75u,  0u,   130u )
#define BLUE                        RGB888( 0u,   0u,   255u )
#define GREEN                       RGB888( 0u,   255u, 0u )
#define YELLOW                      RGB888( 255u, 255u, 0u )
#define ORANGE                      RGB888( 255u, 127u, 0u )
#define RED                         RGB888( 255u, 0u,   0u )
#define WHITE                       RGB888( 255u, 255u, 255u )
#define BLACK                       RGB888( 0u,   0u,   0u )

extern GPIO_TypeDef *ltdc_io_ports[];
extern const uint8_t ltdc_pins[];
extern const uint8_t total_ltdc_pins;

void BSP_LCD_Init( void );
uint32_t BSP_LCD_Get_FB_Address( void );
void BSP_LCD_SetFrameBuffer_BackGroundColor( uint32_t rgb888 );

#endif /* BSP_LCD_H_ */
