#include "GUI.h"

/*-----------------------------Private Functions------------------------------*/
static void custom_delay( void );
static void test_print_text( void );
static void test_draw_basic_routine( void );
static void test_draw_rectangle( void );
static void test_draw_bitmap( void );
static void test_draw_line( void );
static void test_draw_polygon( void );
static void test_draw_circle( void );
static void test_draw_ellipses( void );
static void test_draw_graph( void );
static void test_draw_pie_chart( void );

void MainTask(void) 
{
  test_print_text();
  while(1);
}


/*----------------------Private Functions Definition--------------------------*/
/**
 * @function custom_delay
 * @brief User Delay of fixed value
 */
static void custom_delay( void )
{
  GUI_Delay(1000);
}

/**
 * @function test_print_text
 * @brief Display Text on the Screen
 */
static void test_print_text( void )
{
  GUI_Clear();
  GUI_SetFont(&GUI_Font16_1);
  GUI_SetColor(GUI_LIGHTCYAN);
  GUI_DispStringHCenterAt("2D Graphics Example", LCD_GetXSize()/2, 0);
  GUI_GotoXY(0, 32);
  GUI_DispString("LCD Size X = ");
  GUI_DispDec( LCD_GetXSize(), 3 );
  GUI_DispString(", LCD Size Y = ");
  GUI_DispDec( LCD_GetYSize(), 3 );
  custom_delay();
}

/*************************** End of file ****************************/
