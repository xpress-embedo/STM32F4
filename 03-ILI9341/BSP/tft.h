#ifndef TFT_H_

#include <stdint.h>

#define TFT_HOR_RES             (320u)
#define TFT_VER_RES             (240u)

void TFT_Init( void );
uint16_t TFT_GetWidth( void );
uint16_t TFT_GetHeight( void );

#endif  // TFT_H_
