/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stdint.h>
#include "project_refs.h"
#include "stm32f429xx.h"
#include "bsp_lcd.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

void SystemClock_Setup( void );
void LTDC_Pin_Init( void );
void LTDC_Init( void );

int main(void)
{
  // Setup Clock
  SystemClock_Setup();
  // Initialize LCD
  BSP_LCD_Init();
  LTDC_Pin_Init();
  LTDC_Init();

	for(;;);
}

/**
 * @brief Initializes the STM32 Clock, enable the PLL for system clock and also
 *        for the LTDC which is done using PLLSAI
 * @param  none
 */
void SystemClock_Setup( void )
{
  /*---First Step is to Setup the main system clock SYSCLK, we will configure
  the board at maximum clock, for the board it is 180MHz---*/
  /*---We will use HSI clock which is enabled by default and is 16MHz, and then
  this is fed to PLL module by the divider "M", the PLL block will multiple this
  with "N", and the output produced can be further divided by "P".
  Note: The LTDC peripheral has a separate PLL module, and it's input is HSI/M
  so we have choose "M" properly, so basically the HSI/M output is input for
  PLL VCO (System Clock) and PLLSAI VCO (LTDC Clock)---*/

  RCC_TypeDef *pRCC = RCC;
  FLASH_TypeDef *pFlash = FLASH;
  PWR_TypeDef *pPWR = PWR;

  // Program Flash Wait States
  /*---Since we are going to increase the system clock to a very high frequency
  we need to adjust the flash wait states to read the data/next instruction
  properly from the flash, check datasheet for more information ---*/
  REG_SET_VAL( pFlash->ACR, 0x5, 0xF, FLASH_ACR_LATENCY_Pos );

  // We have to enable the Over Drive mode because we wanted to achieve 180MHz
  // and this is done using the micro-controllers power register
  SET_BIT( pRCC->APB1ENR, RCC_APB1ENR_PWREN_Pos );        // Enable Clock for PWR Register access
  REG_SET_VAL( pPWR->CR, 0x03, 0x03, PWR_CR_VOS_Pos );    // VOS = 0b11
  SET_BIT( pPWR->CR, PWR_CR_ODEN_Pos );                   // Activate Over Drive Mode
  while( ! READ_BIT(pPWR->CSR, PWR_CSR_ODRDY_Pos) );      // Wait for Over Drive Ready
  SET_BIT( pPWR->CR, PWR_CR_ODSWEN_Pos );                 // Over Drive Switch Enable

  // Setup the main clock
  REG_SET_VAL( pRCC->PLLCFGR, 0x08, 0x03F, RCC_PLLCFGR_PLLM_Pos );    		// PLL_M
  REG_SET_VAL( pRCC->PLLCFGR, 180u, 0x1FF, RCC_PLLCFGR_PLLN_Pos );    		// PLL_N
  REG_SET_VAL( pRCC->PLLCFGR, 0x00, 0x03,  RCC_PLLCFGR_PLLP_Pos );    		// PLL_P

  // Enable DOTCLK (Required only when using the RGB Interface)
  REG_SET_VAL( pRCC->PLLSAICFGR, 50u, 0x1FF, RCC_PLLSAICFGR_PLLSAIN_Pos); // PLLSAI_N
  REG_SET_VAL( pRCC->PLLSAICFGR, 0x02, 0x07, RCC_PLLSAICFGR_PLLSAIR_Pos); // PLLSAI_R
  // LCD Clock 6.25MHz
  REG_SET_VAL( pRCC->DCKCFGR, 0x02, 0x3, RCC_DCKCFGR_PLLSAIDIVR_Pos);     // DIV

  SET_BIT( pRCC->CR, RCC_CR_PLLSAION_Pos );     // Enable PLLSAI
  // Stay in loop until ready flag is set
  while( ! READ_BIT( pRCC->CR, RCC_CR_PLLSAIRDY_Pos) );

  // Setup AHB and APBx Clocks
  // There are three bus clocks, HCLK, PCLK1 and PCLK2
  // HCLK is controlled by AHB Precalar and maximum value is 180MHz
  // PCLK1 is controlled by APB1 Presclar and maximum value is 45MHz
  // PCLK2 is controlled by APB2 Presclar and maximum value is 90MHz
  REG_SET_VAL( pRCC->CFGR, 0u, 0xF, RCC_CFGR_HPRE_Pos);
  REG_SET_VAL( pRCC->CFGR, 0x5, 0x7, RCC_CFGR_PPRE1_Pos);
  REG_SET_VAL( pRCC->CFGR, 0x4, 0x7, RCC_CFGR_PPRE2_Pos);

  // Turn on the PLL and wait for PLLCLK Ready
  SET_BIT( pRCC->CR, RCC_CR_PLLON_Pos );
  while( !READ_BIT(pRCC->CR,RCC_CR_PLLRDY_Pos) );

  // Switch PLLCLK as SYSCLK
  REG_SET_VAL( pRCC->CFGR, 0x02, 0x03, RCC_CFGR_SW_Pos );
  while( !(REG_READ_VAL( pRCC->CFGR, 0x3, RCC_CFGR_SWS_Pos) == 0x02 ) );
}

/**
 * @brief Initializes the GPIO's used for LTDC peripheral
 * @param  none
 */
void LTDC_Pin_Init( void )
{
  uint32_t idx = 0;
  RCC_TypeDef *pRCC = RCC;

  // Enable the Peripheral Clocks for the GPIO Ports involved in LTDC Interface
  SET_BIT( pRCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN_Pos );
  SET_BIT( pRCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN_Pos );
  SET_BIT( pRCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN_Pos );
  SET_BIT( pRCC->AHB1ENR, RCC_AHB1ENR_GPIODEN_Pos );
  SET_BIT( pRCC->AHB1ENR, RCC_AHB1ENR_GPIOFEN_Pos );
  SET_BIT( pRCC->AHB1ENR, RCC_AHB1ENR_GPIOGEN_Pos );

  for( idx=0; idx<total_ltdc_pins; idx++ )
  {
    REG_SET_VAL( ltdc_io_ports[idx]->MODER, 0x02, 0x03, (ltdc_pins[idx]*2u) );    // Alternate Function Mode
    CLR_BIT( ltdc_io_ports[idx]->OTYPER, ltdc_pins[idx] );                        // Output Type
    REG_SET_VAL( ltdc_io_ports[idx]->OSPEEDR, 0x02, 0x03, (ltdc_pins[idx]*2u) );  // Speed
    if( ltdc_pins[idx] < 8u )
    {
      REG_SET_VAL( ltdc_io_ports[idx]->AFR[0], 14u, 0x0F, (ltdc_pins[idx]*4u) );  // Alternate Function LTDC
    }
    else
    {
      REG_SET_VAL( ltdc_io_ports[idx]->AFR[1], 14u, 0x0F, ((ltdc_pins[idx]%8)*4u) );  // Alternate Function LTDC
    }
  }
}

void LTDC_Init( void )
{
  RCC_TypeDef *pRCC = RCC;
  LTDC_TypeDef *pLTDC = LTDC;
  uint32_t width = 0u;
  uint32_t height = 0u;

  // Enable the Peripheral Clock for the LTDC, and that can be done in APB2
  SET_BIT( pRCC->APB2ENR, RCC_APB2ENR_LTDCEN_Pos );

  // Horizontal Configurations
  REG_SET_VAL( pLTDC->SSCR, (BSP_LCD_HSW - 1u), 0xFFF, LTDC_SSCR_HSW_Pos );                   // Horizontal Synchronization Size Configuration
  REG_SET_VAL( pLTDC->BPCR, (BSP_LCD_HSW + BSP_LCD_HBP - 1u), 0xFFF, LTDC_BPCR_AHBP_Pos );    // Back Porch Configuration
  
  width = (BSP_LCD_HSW + BSP_LCD_HBP + BSP_LCD_ACTIVE_WIDTH - 1u);
  REG_SET_VAL( pLTDC->AWCR, width, 0xFFF, LTDC_AWCR_AAW_Pos );                                // Active Width Configuration

  width = (BSP_LCD_HSW + BSP_LCD_HBP + BSP_LCD_ACTIVE_WIDTH + BSP_LCD_HFP - 1u);
  REG_SET_VAL( pLTDC->TWCR, width, 0xFFF, LTDC_TWCR_TOTALW_Pos );                             // Total Width Configuration

  // Vertical Configurations
  REG_SET_VAL( pLTDC->SSCR, (BSP_LCD_VSW - 1u), 0x7FF, LTDC_SSCR_VSH_Pos );                   // Vertical Synchronization Size Configuration
  REG_SET_VAL( pLTDC->BPCR, (BSP_LCD_VSW + BSP_LCD_VBP - 1u), 0x7FF, LTDC_BPCR_AVBP_Pos );    // Back Porch Configuration
  
  height = (BSP_LCD_VSW + BSP_LCD_VBP + BSP_LCD_ACTIVE_HEIGHT - 1u);
  REG_SET_VAL( pLTDC->AWCR, height, 0x7FF, LTDC_AWCR_AAH_Pos );                               // Active Height Configuration

  height = (BSP_LCD_VSW + BSP_LCD_VBP + BSP_LCD_ACTIVE_HEIGHT + BSP_LCD_VFP - 1u);
  REG_SET_VAL( pLTDC->TWCR, height, 0xFFF, LTDC_TWCR_TOTALH_Pos );                            // Total Height Configuration

  // Configure the Background Color
  REG_SET_VAL( pLTDC->BCCR, 0xFF, 0xFF, LTDC_BCCR_BCRED_Pos );
  // REG_SET_VAL( pLTDC->BCCR, 0x0000FFU, 0xFFFFFF , LTDC_BCCR_BCBLUE_Pos);

  // Default Polarity for Hsync, Vsync, LTDC CLK, DE
  // TODO: XS

  // Enable the LTDC peripheral
  SET_BIT( pLTDC->GCR, LTDC_GCR_LTDCEN_Pos );
}
