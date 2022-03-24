#include "GUI.h"
#include "stdlib.h"

/*-----------------------------Private Functions------------------------------*/
static void custom_delay( void );
static void test_print_text( void );
static void test_draw_basic_routine( void );
static void test_draw_line( void );
static void test_draw_circle( void );
static void test_draw_ellipses( void );
static void test_draw_polygon( void );
static void test_draw_pie_chart( void );
static void test_draw_graph( void );    // TODO:
static void test_draw_bitmap( void );   // TODO:

void MainTask(void) 
{
  test_print_text();
  test_draw_basic_routine();
  test_draw_line();
  test_draw_circle();
  test_draw_ellipses();
  test_draw_polygon();
  test_draw_pie_chart();
  test_draw_graph();
  while(1);
}


/*----------------------Private Functions Definition--------------------------*/
/***************************************************************************//**
 * @function custom_delay
 * @brief User Delay of fixed value
 ******************************************************************************/
static void custom_delay( void )
{
  GUI_Delay(1000);
}

/***************************************************************************//**
 * @function test_print_text
 * @brief Display Text on the Screen
 ******************************************************************************/
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

/***************************************************************************//**
 * @function test_draw_basic_routine
 * @brief Test Basic 2D Drawing Routines
 ******************************************************************************/
static void test_draw_basic_routine( void )
{
  // Basic Drawing Routines
  GUI_Clear();
  // Draw a rectangle with horizontal color gradient
  GUI_DrawGradientH(0, 0, 63, 63, GUI_LIGHTCYAN, GUI_DARKCYAN);
  custom_delay();
  // Draw a rectangle with vertical color gradient
  GUI_DrawGradientV(64, 0, 127, 63, GUI_LIGHTCYAN, GUI_DARKCYAN);
  custom_delay();
  // Draw a rectangle with rounded corners filled with a horizontal color gradient
  GUI_DrawGradientRoundedH(128, 0, 191, 63, 20, GUI_LIGHTBLUE, GUI_DARKBLUE);
  custom_delay();
  // Draw a rectangle with rounded corners filled with a horizontal color gradient
  GUI_DrawGradientRoundedV(0, 64, 63, 127, 20, GUI_LIGHTBLUE, GUI_DARKBLUE);
  custom_delay();
  // Draw a Pixel at a specified position
  GUI_DrawPixel(200, 32);
  custom_delay();
  // Draw a Point with the Current Pen Size
  // At the moment Pen Size is 1, so first we will change the Pen Size
  // Change Pen Size to 10
  GUI_SetPenSize(10);
  GUI_DrawPoint(200, 50);
  
  // Reset the Pen Size back to 1
  GUI_SetPenSize(1);
  custom_delay();
}

/***************************************************************************//**
 * @function test_draw_line
 * @brief Test different Draw Line API
 ******************************************************************************/
static void test_draw_line( void )
{
  // Drawing Lines
  GUI_Clear();
  GUI_SetColor(GUI_WHITE);
  // Draw Line (the following two lines will draw diagonal lines
  GUI_DrawLine(0, 0, LCD_GetXSize(), LCD_GetYSize() );
  GUI_DrawLine(LCD_GetXSize(), 0, 0, LCD_GetYSize() );
  GUI_SetColor(GUI_RED);
  // Draw Horizontal Line with 1 pixel thickness
  GUI_DrawHLine( LCD_GetYSize()/2, 0, LCD_GetXSize() );
  GUI_SetColor(GUI_BLUE);
  // Draw Vertical Line
  GUI_DrawVLine( LCD_GetXSize()/2, 0, LCD_GetYSize() );
  
  GUI_SetColor(GUI_YELLOW);
  // Line Style: Only works with GUI_DrawLine function
  GUI_SetLineStyle(GUI_LS_DASH);
  GUI_DrawLine(0, 0, LCD_GetXSize()/2, LCD_GetYSize() );
  GUI_Delay(50);
  GUI_SetLineStyle(GUI_LS_DOT);
  GUI_DrawLine(0, 0, LCD_GetXSize()/3, LCD_GetYSize() );
  GUI_Delay(50);
  GUI_SetLineStyle(GUI_LS_DASHDOT);
  GUI_DrawLine(0, 0, LCD_GetXSize()/4, LCD_GetYSize() );
  GUI_Delay(50);
  GUI_SetLineStyle(GUI_LS_DASHDOTDOT);
  GUI_DrawLine(0, 0, LCD_GetXSize()/5, LCD_GetYSize() );
  custom_delay();
}

/***************************************************************************//**
 * @function test_draw_circle
 * @brief Test Circle Draw APIs
 ******************************************************************************/
static void test_draw_circle( void )
{
  // Drawing Circles
  int i;
  GUI_Clear();
  GUI_SetColor(GUI_LIGHTCYAN);
  // Filled Circle
  GUI_SetColor(GUI_DARKMAGENTA);
  GUI_FillCircle( LCD_GetXSize()/2, LCD_GetYSize()/2, 20 );
  for( i=30; i< LCD_GetXSize()/2; i+=10 )
  {
    // Normal Circle
    GUI_DrawCircle( LCD_GetXSize()/2, LCD_GetYSize()/2, i);
  }
  custom_delay();
}

/***************************************************************************//**
 * @function test_draw_ellipses
 * @brief Test Ellipse Draw APIs
 ******************************************************************************/
static void test_draw_ellipses( void )
{
  /// Drawing Ellipses ///
  GUI_Clear();
  GUI_SetColor(GUI_RED);
  GUI_FillEllipse(100, 180, 50, 70);
  GUI_SetColor(GUI_MAGENTA);
  GUI_DrawEllipse(100, 180, 50, 70);
  GUI_SetColor(GUI_YELLOW);
  GUI_FillEllipse(100, 180, 10, 50);
  custom_delay();
}

/***************************************************************************//**
 * @function test_draw_polygon
 * @brief Test Polygon Draw APIs
 ******************************************************************************/
static void test_draw_polygon( void )
{
  // Drawing Polygons
  int i;
  GUI_Clear();
  const GUI_POINT aPoints[] = {
    { 40, 20 },
    {  0, 20 },
    { 20,  0 }
  };
  // This will be used to save the source pointer
  GUI_POINT aEnlargedPoints[GUI_COUNTOF(aPoints)];
  GUI_SetColor(GUI_LIGHTBLUE);
  GUI_FillPolygon( aPoints, GUI_COUNTOF(aPoints), 0, 0 );
  // TODO: XE, this needs to be checked why xoring is not working
  // due to this when polygon are enlarged it is overwritten
  // GUI_SetDrawMode(GUI_DM_XOR);
  GUI_FillPolygon(aPoints, GUI_COUNTOF(aPoints), 140, 110);
  GUI_SetColor(GUI_WHITE);
  for( i=1; i<10; i++ )
  {
    GUI_EnlargePolygon(aEnlargedPoints, aPoints, GUI_COUNTOF(aPoints), i*5 );
    GUI_FillPolygon(aEnlargedPoints, GUI_COUNTOF(aPoints), 140, 110);
  }
  // Reset Drawing Mode back to Normal
  GUI_SetDrawMode(GUI_DRAWMODE_NORMAL);
  custom_delay();
}

/***************************************************************************//**
 * @function test_draw_pie_chart
 * @brief Test PIE Chart API's
 ******************************************************************************/
static void test_draw_pie_chart( void )
{
  int i, start_angle, stop_angle;
  const unsigned aValues[] = { 100, 135, 190, 240, 340, 360};
  const GUI_COLOR aColors[] = { GUI_BLUE, GUI_GREEN, GUI_RED, GUI_CYAN, GUI_MAGENTA, GUI_YELLOW };
  GUI_Clear();
  for (i = 0; i < GUI_COUNTOF(aValues); i++)
  {
    start_angle = (i == 0) ? 0 : aValues[i - 1];
    stop_angle = aValues[i];
    GUI_SetColor(aColors[i]);
    GUI_DrawPie(LCD_GetXSize()/2, LCD_GetYSize()/2, LCD_GetYSize()/3, start_angle, stop_angle, 0);
  }
  custom_delay();
}

/***************************************************************************//**
 * @function test_draw_graph
 * @brief Test Graph API's
 ******************************************************************************/
static void test_draw_graph( void )
{
  int x[240] = { 0 };   // Size of X-Axis
  int i = 0;
  GUI_Clear();
  GUI_SetColor(GUI_RED);
  for (i = 0; i < GUI_COUNTOF(x); i++)
  {
    x[i] = rand() % LCD_GetYSize()/2;
  }
  // GUI_DrawGraph(x, GUI_COUNTOF(x), 0, 0 );
  custom_delay();
}

/*************************** End of file ****************************/
