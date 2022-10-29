/*
 * bsp_lcd.h
 *
 *  Created on: 26-Oct-2022
 *      Author: xpress_embedo
 */

#ifndef BSP_LCD_H_
#define BSP_LCD_H_

#include "project_refs.h"
#include "stm32f429xx.h"

#define PORTRAIT                    (0u)
#define LANSCAPE                    (1u)
#define ORIENTATION                 (LANSCAPE)

#define BSP_LCD_HSW                 (10u)   // Horizontal Synch Width
#define BSP_LCD_HBP                 (20u)   // Horizontal Back Porch
#define BSP_LCD_HFP                 (10u)   // Horizontal Front Porch

#define BSP_LCD_VSW                 (2u)    // Vertical Synch Width
#define BSP_LCD_VBP                 (2u)    // Vertical Back Porch
#define BSP_LCD_VFP                 (4u)    // Vertical Front Porch

#if (ORIENTATION == LANSCAPE )
#define BSP_LCD_ACTIVE_WIDTH        (240u)
#define BSP_LCD_ACTIVE_HEIGHT       (320u)
#elif (ORIENTATION == PORTRAIT )
#define BSP_LCD_ACTIVE_WIDTH        (320u)
#define BSP_LCD_ACTIVE_HEIGHT       (240u)
#else
#error "Orientation Not Selected."
#endif

extern GPIO_TypeDef *ltdc_io_ports[];
extern const uint8_t ltdc_pins[];
extern const uint8_t total_ltdc_pins;

void BSP_LCD_Init( void );

#endif /* BSP_LCD_H_ */
