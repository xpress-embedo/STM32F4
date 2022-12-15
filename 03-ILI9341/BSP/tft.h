#ifndef TFT_H_

#include <stdint.h>

#define SPI_DATA_TRANSFER_8BIT        (0u)
#define SPI_DATA_TRANSFER_16BIT       (1u)

// #define SPI_DATA_TRANSFER_BITS        SPI_DATA_TRANSFER_8BIT
#define SPI_DATA_TRANSFER_BITS        SPI_DATA_TRANSFER_16BIT

#define TFT_HOR_RES             (320u)
#define TFT_VER_RES             (240u)

void TFT_Init( void );
uint16_t TFT_GetWidth( void );
uint16_t TFT_GetHeight( void );

#endif  // TFT_H_
