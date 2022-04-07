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

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

/*
 * In this example we will do the HSE (High Speed External Oscillator) Clock
 * Measurement, these clocks whether HSI or HSE can be connected to MCO1, which
 * is micro-controller output-1,and then we can use GPIO's alternate function
 * to connect this MCO1 output to PA8 pin of GPIOA.
 * It should also output 8MHz (8MHz is external crystal on this discovery board)
 * so it should output the same frequency, but since the Logic Analyzer which I
 * am using is pretty slow, I will pre-scale the output by 4, and will only get
 * 2MHz on the PA8 pin of GPIOA
 */

#define RCC_BASE_ADDR             (0x40023800ul)    // Reset & Control Register Base Address
#define RCC_CR_OFSET              (0x00ul)          // Offset Value for RCC Clock Control Register
#define RCC_PLLCFGCR_OFSET        (0x04ul)          // Offset Value for RCC PLL Clock Configuration Register
#define RCC_CFGR_OFSET            (0x08ul)          // Offset Value for RCC Clock Configuration Register
#define RCC_AHB1ENR_OFSET         (0x30ul)          // Offset Value for RCC AHB1 Peripheral Clock Register

#define RCC_CR_ADDR               (RCC_BASE_ADDR + RCC_CR_OFSET)      // Address of RCC Clock Control Register
#define RCC_CFGR_ADDR             (RCC_BASE_ADDR + RCC_CFGR_OFSET)    // Address of RCC Clock Configuration Register
#define RCC_AHB1ENR_ADDR          (RCC_BASE_ADDR + RCC_AHB1ENR_OFSET) // Address of AHB1 Peripheral Clock Register

#define GPIOA_BASE_ADDR           (0x40020000ul)    // GPIOA Base Address
#define GPIOA_MODER_OFSET         (0x00ul)          // GPIOA Port Mode Register Offset
#define GPIOA_AFRL_OFSET          (0x20ul)          // GPIOA Alternate Function Low Register Offset
#define GPIOA_AFRH_OFSET          (0x24ul)          // GPIOA Alternate Function High Register Offset

#define GPIOA_MODER_ADDR          (GPIOA_BASE_ADDR + GPIOA_MODER_OFSET) // GPIOA Port Mode Register Address
#define GPIOA_AFRL_ADDR           (GPIOA_BASE_ADDR + GPIOA_AFRL_OFSET)  // GPIOA Alternate Function Low Register Address
#define GPIOA_AFRH_ADDR           (GPIOA_BASE_ADDR + GPIOA_AFRH_OFSET)  // GPIOA Alternate Function High Register Address


int main(void)
{
  // 0) Enable the HSE clock using HSEON bit
  uint32_t *p_rcc_cr_reg = (uint32_t*)RCC_CR_ADDR;
  *p_rcc_cr_reg |= (0x01 << 16u);   // HSE bit is turned on

  // 1) But now we have to wait until HSE Clock Stabilizes
  // This can be checked by the status of HSERDY, HSE Clock Ready Flag
  while ( !(*p_rcc_cr_reg & (0x01<<17u)) );

  // 2) Switch the clock to HSE Clock (Here we are switching from HSI to HSE)
  uint32_t *p_rcc_cfg_reg = (uint32_t*)(RCC_CFGR_ADDR);
  *p_rcc_cfg_reg |= (0x01 << 0u);

  // 3) Configure the RCC Clock Configuration Register MCO1 bit fields to select
  // the HSE as Clock Source (HSI is internal and by default clock source of the
  // controller and for this controller it's value is 16 MHz, but here we are
  // configuring the HSE clock source)
  // NOTE: MCO1 means micro-controller clock output-1 Bits 22:21
  // 00 -> HSI Clock is selected
  // 01 -> LSE oscillator is selected
  // 10 -> HSE oscillator is selected
  // 11 -> PLL clock is selected
  // Since we want to output HSE clock on MCO1, we will do the following
  *p_rcc_cfg_reg &= ~(0x03 << 21u);   // Clear the 21 and 22 bits
  *p_rcc_cfg_reg |= (0x01 << 22u);    // Set the 22 bit

  // Ideally we should see 16MHz on Logic Analyzer or on Oscilloscope
  // In my case I have USB Logic Analyzer of 24MHz, which is not capable to
  // capture this signal, so I will use the pre-scaler to scale down this output
  // frequency to a lower value.
  // Configure the MCO1 Pre-scaler to divide by 4
  // Bit 26:25:24
  // 000 --> No Division
  // 100 --> Division by 2
  // 101 --> Division by 3
  // 110 --> Division by 4 --> This is set, this means output frequency should be 4MHz
  // 111 --> Division by 5
  *p_rcc_cfg_reg |= (0x01 << 25u);
  *p_rcc_cfg_reg |= (0x01 << 26u);

  // 4) MCO1 signal is internal to controller, and to see this signal we need to
  // output this signal on a GPIO pin.
  // As per datasheet of the controller, PA8 pin can be used as MCO1 pin, when
  // configured as AF0 i.e. alternate functionality 0
  // But to configure this, first of all we need to enable the clock for the PA8
  // of the controller which is on GPIOA port.
  // GPIOA is on AHB1 bus, and to enable it we need to update the RCC_AHB1ENR
  uint32_t *p_rcc_ahb1_enr = (uint32_t*)(RCC_AHB1ENR_ADDR);
  *p_rcc_ahb1_enr |= (0x01 << 0u);    // Set Bit-0, which enables the GPIOA Peripheral Clock

  // Now, the clock is enabled, we need to configure the alternate function on
  // PA8 pin GPIOA as AF0
  uint32_t *p_gpioa_mode_reg = (uint32_t*)(GPIOA_MODER_ADDR);
  // First we need to make this pin in alternate function mode, this can be done
  // by writing 10 on MODERy[1:0] bit, where y is from 0 to 15
  // Since we need to update the PA8, we will write on 16th bits
  *p_gpioa_mode_reg &= ~(0x03 << 16u);  // first clear these bits
  *p_gpioa_mode_reg |= (0x02 << 16u);   // now update for 10 value for Alternate Function Mode

  // Now configure PA8 for AF0 alternate function, and to update this we have to
  // use the GPIOA_AFRH_ADDR , and to configure it as AF0, we need to write 0x0
  // we need to clear the bits
  uint32_t *p_gpioa_amode_reg = (uint32_t*)(GPIOA_AFRH_ADDR);
  *p_gpioa_amode_reg &= ~(0xF << 0u);

  /* Loop forever */
  for(;;);
}
