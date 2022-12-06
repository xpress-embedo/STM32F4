/*
 * display_mng.c
 *
 *  Created on: 05-Dec-2022
 *      Author: xpress_embedo
 */

#include "display_mng.h"
#include "tft.h"
#include "lvgl/lvgl.h"

#include <stdio.h>
uint32_t dbg_size = 0u;
char dbg_buff[100] = { 0 };
extern UART_HandleTypeDef huart2;

typedef enum _Display_State_e
{
  DISP_STATE_VIBGYOR = 0,
  DISP_STATE_VIBGYOR_WAIT,
  DISP_STATE_RGB_MIXER,
  DISP_STATE_END,
} Display_State_e;

typedef enum _SliderType_e
{
  SLIDER_TYPE_RED = 0,
  SLIDER_TYPE_GREEN,
  SLIDER_TYPE_BLUE,
} SliderType_e;

typedef struct _RGB_Mixer_s
{
  SliderType_e  slider_type;
  lv_obj_t*     label;
} RGB_Mixer_s;


static Display_State_e disp_state = DISP_STATE_RGB_MIXER; //DISP_STATE_VIBGYOR;
static RGB_Mixer_s red, green, blue;
static lv_obj_t *slider_r;
static lv_obj_t *slider_g;
static lv_obj_t *slider_b;
static lv_obj_t *rectangle;

static void Display_Vibgyor( void );
static void Display_RGBMixer( void );
static void Slider_DummyCallback( RGB_Mixer_s *user_data, int32_t slider_value );

// Display manager state machine
void Display_Mng( void )
{
  static uint32_t wait_time = 0u;
  switch( disp_state )
  {
    case DISP_STATE_VIBGYOR:
      Display_Vibgyor();
      wait_time = HAL_GetTick();
      disp_state = DISP_STATE_VIBGYOR_WAIT;
      break;
    case DISP_STATE_VIBGYOR_WAIT:
      // wait here for some time and then move to next state
      if( HAL_GetTick()-wait_time > 1000u )
      {
        disp_state = DISP_STATE_RGB_MIXER;
      }
      break;
    case DISP_STATE_RGB_MIXER:
      Display_RGBMixer();
      disp_state = DISP_STATE_END;
      break;
    case DISP_STATE_END:
      // dummy code
//      red.slider_type = SLIDER_TYPE_RED;
//      green.slider_type = SLIDER_TYPE_GREEN;
//      blue.slider_type = SLIDER_TYPE_BLUE;
      Slider_DummyCallback( &red, 255 );
      Slider_DummyCallback( &green, 10 );
      Slider_DummyCallback( &blue, 255 );
      break;
    default:
      // don't do anything here
      break;
  };
}

static void Display_Vibgyor( void )
{
  static lv_style_t style;
//  lv_coord_t x_ofset = 0;
//  lv_coord_t y_ofset = 0;
  lv_coord_t width = 0u;
  lv_coord_t length = 0u;

  lv_obj_t * V_rectangle;
  lv_obj_t * I_rectangle;
  lv_obj_t * B_rectangle;
  lv_obj_t * G_rectangle;
  lv_obj_t * Y_rectangle;
  lv_obj_t * O_rectangle;
  lv_obj_t * R_rectangle;

  lv_obj_t *act_scr = lv_scr_act();           // Get the active screen object

  R_rectangle = lv_obj_create( act_scr );     // Create Rectangle Object
  O_rectangle = lv_obj_create( act_scr );
  Y_rectangle = lv_obj_create( act_scr );
  G_rectangle = lv_obj_create( act_scr );
  B_rectangle = lv_obj_create( act_scr );
  I_rectangle = lv_obj_create( act_scr );
  V_rectangle = lv_obj_create( act_scr );

  lv_style_init(&style);
  // set the radius to zero
  lv_style_set_radius(&style, 0);
  // by default the object which we created for rectangle has some radius component
  // and it looks bad for this particular example, hence updating style for all
  // created objects
  lv_obj_add_style(R_rectangle, &style, 0);
  lv_obj_add_style(O_rectangle, &style, 0);
  lv_obj_add_style(Y_rectangle, &style, 0);
  lv_obj_add_style(G_rectangle, &style, 0);
  lv_obj_add_style(B_rectangle, &style, 0);
  lv_obj_add_style(I_rectangle, &style, 0);
  lv_obj_add_style(V_rectangle, &style, 0);


  length = (lv_coord_t)TFT_GetWidth();
  // VIBGYOR are seven colors, height of display is 240,
  // one color height is 240/7 = 34
  width = (lv_coord_t)(TFT_GetHeight()/7u);

  // memset( dbg_buff, 0x00, 100u );
  // dbg_size = snprintf(dbg_buff, 100u, "length=%d, width=%d\r\n", (uint16_t)length, (uint16_t)width);
  // HAL_UART_Transmit(&huart2, (uint8_t*)dbg_buff, dbg_size, 1000u);
  
  lv_obj_set_size(R_rectangle, length, width);
  lv_obj_align(R_rectangle, LV_ALIGN_TOP_LEFT, 0, 0 );
  lv_obj_set_style_border_color(R_rectangle, lv_palette_main(LV_PALETTE_RED), LV_PART_MAIN );
  lv_obj_set_style_bg_color( R_rectangle, lv_palette_main(LV_PALETTE_RED), LV_PART_MAIN );

  lv_obj_set_size(O_rectangle, length, width );
  // lv_obj_align_to(O_rectangle, R_rectangle, LV_ALIGN_BOTTOM_MID, 0, 0);
  lv_obj_align(O_rectangle, LV_ALIGN_TOP_LEFT, 0, width );
  lv_obj_set_style_border_color(O_rectangle, lv_palette_main(LV_PALETTE_ORANGE), LV_PART_MAIN );
  lv_obj_set_style_bg_color( O_rectangle, lv_palette_main(LV_PALETTE_ORANGE), LV_PART_MAIN );

  lv_obj_set_size(Y_rectangle, length, width );
  // lv_obj_align_to(Y_rectangle, O_rectangle, LV_ALIGN_TOP_LEFT, 0, 0);
  lv_obj_align(Y_rectangle, LV_ALIGN_TOP_LEFT, 0, width*2u );
  lv_obj_set_style_border_color(Y_rectangle, lv_palette_main(LV_PALETTE_YELLOW), LV_PART_MAIN );
  lv_obj_set_style_bg_color( Y_rectangle, lv_palette_main(LV_PALETTE_YELLOW), LV_PART_MAIN );

  lv_obj_set_size(G_rectangle, length, width );
  // lv_obj_align_to(G_rectangle, Y_rectangle, LV_ALIGN_TOP_LEFT, 0, 0);
  lv_obj_align(G_rectangle, LV_ALIGN_TOP_LEFT, 0, width*3u );
  lv_obj_set_style_border_color(G_rectangle, lv_palette_main(LV_PALETTE_GREEN), LV_PART_MAIN );
  lv_obj_set_style_bg_color( G_rectangle, lv_palette_main(LV_PALETTE_GREEN), LV_PART_MAIN );

  lv_obj_set_size(B_rectangle, length, width );
  // lv_obj_align_to(B_rectangle, G_rectangle, LV_ALIGN_TOP_LEFT, 0, 0);
  lv_obj_align(B_rectangle, LV_ALIGN_TOP_LEFT, 0, width*4u );
  lv_obj_set_style_border_color(B_rectangle, lv_palette_main(LV_PALETTE_BLUE), LV_PART_MAIN );
  lv_obj_set_style_bg_color( B_rectangle, lv_palette_main(LV_PALETTE_BLUE), LV_PART_MAIN );

  lv_obj_set_size(I_rectangle, length, width );
  // lv_obj_align_to(Y_rectangle, B_rectangle, LV_ALIGN_TOP_LEFT, 0, 0);
  lv_obj_align(I_rectangle, LV_ALIGN_TOP_LEFT, 0, width*5u );
  lv_obj_set_style_border_color(I_rectangle, lv_palette_main(LV_PALETTE_INDIGO), LV_PART_MAIN );
  lv_obj_set_style_bg_color( I_rectangle, lv_palette_main(LV_PALETTE_INDIGO), LV_PART_MAIN );

  lv_obj_set_size(V_rectangle, length, width );
  // lv_obj_align_to(V_rectangle, I_rectangle, LV_ALIGN_TOP_LEFT, 0, 0);
  lv_obj_align(V_rectangle, LV_ALIGN_TOP_LEFT, 0, width*6u );
  lv_obj_set_style_border_color(V_rectangle, lv_palette_main(LV_PALETTE_DEEP_PURPLE), LV_PART_MAIN );
  lv_obj_set_style_bg_color( V_rectangle, lv_palette_main(LV_PALETTE_DEEP_PURPLE), LV_PART_MAIN );
}

static void Display_RGBMixer( void )
{
  lv_obj_t *act_scr = lv_scr_act();                     // Get the active screen object

  lv_obj_clean( act_scr );                              // Clean the screen

  // RED Slider Configuration
  // Slider has three parts Main, Indicator and Knob
  slider_r = lv_slider_create( act_scr );               // create a red slider base object
  lv_obj_align( slider_r, LV_ALIGN_TOP_MID, 0u, 30u);
  // apply red color to the indicator part
  lv_obj_set_style_bg_color( slider_r, lv_palette_main(LV_PALETTE_RED), LV_PART_INDICATOR );
  // apply red color to the knob part
  lv_obj_set_style_bg_color( slider_r, lv_palette_main(LV_PALETTE_RED), LV_PART_KNOB );

  // Green Slider Configuration
  slider_g = lv_slider_create( act_scr );               // create a green slider base object
  lv_obj_align_to( slider_g, slider_r, LV_ALIGN_TOP_MID, 0u, 50u );
  // apply green color to the indicator part
  lv_obj_set_style_bg_color( slider_g, lv_palette_main(LV_PALETTE_GREEN), LV_PART_INDICATOR );
  // apply green color to the knob part
  lv_obj_set_style_bg_color( slider_g, lv_palette_main(LV_PALETTE_GREEN), LV_PART_KNOB );

  // BLUE Slider Configuration
  slider_b = lv_slider_create( act_scr );               // create a blue slider base object
  lv_obj_align_to( slider_b, slider_g, LV_ALIGN_TOP_MID, 0u, 50u );
  // apply blue color to the indicator part
  lv_obj_set_style_bg_color( slider_b, lv_palette_main(LV_PALETTE_BLUE), LV_PART_INDICATOR );
  // apply blue color to the knob part
  lv_obj_set_style_bg_color( slider_b, lv_palette_main(LV_PALETTE_BLUE), LV_PART_KNOB );

  rectangle = lv_obj_create(act_scr);                   // Creates a base object Rectangle to display color
  lv_obj_set_size( rectangle, 300u, 60u );
  lv_obj_align_to( rectangle, slider_b, LV_ALIGN_TOP_MID, 0u, 30u );
  lv_obj_set_style_border_color( rectangle, lv_color_black(), LV_PART_MAIN );   // add black border to rectangle
  lv_obj_set_style_border_width( rectangle, 2, LV_PART_MAIN );                  // increase the width of the border by 2px

  // Create Main Heading Label
  lv_obj_t *heading = lv_label_create(act_scr);
  lv_label_set_text( heading, "RGB Mixer");
  lv_obj_align( heading, LV_ALIGN_TOP_MID, 0u, 1u );

  // Creating labels for individual slider current values
  red.slider_type = SLIDER_TYPE_RED;
  red.label = lv_label_create(act_scr);
  lv_label_set_text( red.label, "0");
  // lv_obj_align_to( red.label, slider_r, LV_ALIGN_TOP_MID, 0u, 0u );         // this will display inside slider
  lv_obj_align_to( red.label, slider_r, LV_ALIGN_OUT_TOP_MID, 0u, 0u );    // this will display outside slider

  green.slider_type = SLIDER_TYPE_GREEN;
  green.label = lv_label_create(act_scr);
  lv_label_set_text( green.label, "0");
  lv_obj_align_to( green.label, slider_g, LV_ALIGN_OUT_TOP_MID, 0u, 0u );

  blue.slider_type = SLIDER_TYPE_BLUE;
  blue.label = lv_label_create(act_scr);
  lv_label_set_text( blue.label, "0");
  lv_obj_align_to( blue.label, slider_b, LV_ALIGN_OUT_TOP_MID, 0u, 0u );

  // add event callbacks for sliders
  // lv_obj_add_event_cb( slider_r, slider_callback, LV_EVENT_VALUE_CHANGED, &red );
  // lv_obj_add_event_cb( slider_g, slider_callback, LV_EVENT_VALUE_CHANGED, &green );
  // lv_obj_add_event_cb( slider_b, slider_callback, LV_EVENT_VALUE_CHANGED, &blue );

  // set slider range also (by default it is 0 to 100 but we want till 255)
  lv_slider_set_range( slider_r, 0, 255 );
  lv_slider_set_range( slider_g, 0, 255 );
  lv_slider_set_range( slider_b, 0, 255 );
}

static void Slider_DummyCallback( RGB_Mixer_s *user_data, int32_t slider_value )
{
  static int32_t red_value, green_value, blue_value;

  // now we have to update the slider labels
  lv_label_set_text_fmt( user_data->label, "%ld", slider_value );

  // now we have to update the color of the rectangle object based on the current
  // selected values of the color
  if( user_data->slider_type == SLIDER_TYPE_RED )
  {
    red_value = slider_value;
    lv_slider_set_value( slider_r, slider_value, LV_ANIM_OFF);
  }
  else if( user_data->slider_type == SLIDER_TYPE_GREEN )
  {
    green_value = slider_value;
    lv_slider_set_value( slider_g, slider_value, LV_ANIM_OFF);
  }
  else if( user_data->slider_type == SLIDER_TYPE_BLUE )
  {
    blue_value = slider_value;
    lv_slider_set_value( slider_b, slider_value, LV_ANIM_OFF);
  }
  // now we have the color information, update the rectangle color
  // NOTE: rectangle object must be file global else we will not be able to
  // update it from here
  lv_obj_set_style_bg_color( rectangle, lv_color_make( red_value, green_value, blue_value), LV_PART_MAIN);
  // the function `lv_color_make` can form colors if red, green and blue color
  // are specified
//  memset( dbg_buff, 0x00, 100u );
//  dbg_size = snprintf(dbg_buff, 100u, "red=%ld, blue=%ld, green=%ld\r\n", red_value, blue_value, green_value );
//  HAL_UART_Transmit(&huart2, (uint8_t*)dbg_buff, dbg_size, 1000u);
}
