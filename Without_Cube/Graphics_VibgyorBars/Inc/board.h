#ifndef BOARD_H_
#define BOARD_H_

#define GPIO_PIN_0                  (0u)
#define GPIO_PIN_1                  (1u)
#define GPIO_PIN_2                  (2u)
#define GPIO_PIN_3                  (3u)
#define GPIO_PIN_4                  (4u)
#define GPIO_PIN_5                  (5u)
#define GPIO_PIN_6                  (6u)
#define GPIO_PIN_7                  (7u)
#define GPIO_PIN_8                  (8u)
#define GPIO_PIN_9                  (9u)
#define GPIO_PIN_10                 (10u)
#define GPIO_PIN_11                 (11u)
#define GPIO_PIN_12                 (12u)
#define GPIO_PIN_13                 (13u)
#define GPIO_PIN_14                 (14u)
#define GPIO_PIN_15                 (15u)

#define LCD_DATA_R2_PIN             GPIO_PIN_10
#define LCD_DATA_R3_PIN             GPIO_PIN_0
#define LCD_DATA_R4_PIN             GPIO_PIN_11
#define LCD_DATA_R5_PIN             GPIO_PIN_12
#define LCD_DATA_R6_PIN             GPIO_PIN_1
#define LCD_DATA_R7_PIN             GPIO_PIN_6

#define LCD_DATA_G2_PIN             GPIO_PIN_6
#define LCD_DATA_G3_PIN             GPIO_PIN_10
#define LCD_DATA_G4_PIN             GPIO_PIN_10
#define LCD_DATA_G5_PIN             GPIO_PIN_11
#define LCD_DATA_G6_PIN             GPIO_PIN_7
#define LCD_DATA_G7_PIN             GPIO_PIN_3

#define LCD_DATA_B2_PIN             GPIO_PIN_6
#define LCD_DATA_B3_PIN             GPIO_PIN_11
#define LCD_DATA_B4_PIN             GPIO_PIN_12
#define LCD_DATA_B5_PIN             GPIO_PIN_3
#define LCD_DATA_B6_PIN             GPIO_PIN_8
#define LCD_DATA_B7_PIN             GPIO_PIN_9

#define LCD_HSYNC_PIN               GPIO_PIN_6
#define LCD_VSYNC_PIN               GPIO_PIN_4
#define LCD_DE_PIN                  GPIO_PIN_10
#define LCD_DOTCLK_PIN              GPIO_PIN_7

#define LCD_DATA_R2_PORT            GPIOC
#define LCD_DATA_R3_PORT            GPIOB
#define LCD_DATA_R4_PORT            GPIOA
#define LCD_DATA_R5_PORT            GPIOA
#define LCD_DATA_R6_PORT            GPIOB
#define LCD_DATA_R7_PORT            GPIOG

#define LCD_DATA_G2_PORT            GPIOA
#define LCD_DATA_G3_PORT            GPIOG
#define LCD_DATA_G4_PORT            GPIOB
#define LCD_DATA_G5_PORT            GPIOB
#define LCD_DATA_G6_PORT            GPIOC
#define LCD_DATA_G7_PORT            GPIOD

#define LCD_DATA_B2_PORT            GPIOD
#define LCD_DATA_B3_PORT            GPIOG
#define LCD_DATA_B4_PORT            GPIOG
#define LCD_DATA_B5_PORT            GPIOA
#define LCD_DATA_B6_PORT            GPIOB
#define LCD_DATA_B7_PORT            GPIOB

#define LCD_HSYNC_PORT              GPIOC
#define LCD_VSYNC_PORT              GPIOA
#define LCD_DE_PORT                 GPIOF
#define LCD_DOTCLK_PORT             GPIOG

#endif