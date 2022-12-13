/**
 * @file disp.h
 * 
 */

#ifndef DISP_H
#define DISP_H

/*********************
 *      INCLUDES
 *********************/
#include <stdint.h>
#include "lvgl/lvgl.h"

/*********************
 *      DEFINES
 *********************/
#define TFT_HOR_RES 240
#define TFT_VER_RES 320

//#define TFT_EXT_FB		1		/*Frame buffer is located into an external SDRAM*/
//#define TFT_USE_GPU		1		/*Enable hardware accelerator*/
#define TFT_EXT_FB    0   /*Frame buffer is located into an internal RAM*/
#define TFT_USE_GPU   0   /*Disable hardware accelerator*/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/
void TFT_Init(void);
uint16_t TFT_GetWidth( void );
uint16_t TFT_GetHeight( void );

/**********************
 *      MACROS
 **********************/

#endif