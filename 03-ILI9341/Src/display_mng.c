/*
 * display_mng.c
 *
 *  Created on: 05-Dec-2022
 *      Author: xpress_embedo
 */

#include "display_mng.h"
#include "tft.h"
#include "lvgl/lvgl.h"

// #include <stdio.h>
// #include "main.h"
// uint32_t dbg_size = 0u;
// char dbg_buff[100] = { 0 };
// extern UART_HandleTypeDef huart2;

typedef enum _Display_State_e
{
  DISP_STATE_VIBGYOR = 0,
  DISP_STATE_END,
} Display_State_e;


static Display_State_e disp_state = DISP_STATE_VIBGYOR;

static void display_vigbyor( void );

// Display manager state machine
void display_mng( void )
{
  switch( disp_state )
  {
    case DISP_STATE_VIBGYOR:
      display_vigbyor();
      disp_state = DISP_STATE_END;
      break;
    case DISP_STATE_END:
    default:
      // don't do anything here
      break;
  };
}

static void display_vigbyor( void )
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


