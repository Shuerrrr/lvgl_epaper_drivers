/**
 * @file disp_driver.h
 */

#ifndef DISP_DRIVER_H
#define DISP_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif

#if defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_ILI9341
#include "ili9341.h"
#elif defined CONFIG_LV_EPAPER_EPDIY_DISPLAY_CONTROLLER
#include "lvgl_tft/epdiy_epaper.h"
#elif defined CONFIG_LV_EPAPER_CALEPD_DISPLAY_CONTROLLER
#include "lvgl_tft/calepd_epaper.h"
#elif defined CONFIG_LV_SHARP_DISPLAY_CONTROLLER
#include "lvgl_tft/sharp_mip.h"

#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_ILI9481
#include "ili9481.h"
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_ILI9488
#include "ili9488.h"
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_ST7789
#include "st7789.h"
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_ST7796S
#include "st7796s.h"
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_ST7735S
#include "st7735s.h"
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_HX8357
#include "hx8357.h"
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_ILI9486
#include "ili9486.h"
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_SH1107
#include "sh1107.h"
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_SSD1306
#include "ssd1306.h"
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_FT81X
#include "FT81x.h"
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_IL3820
#include "il3820.h"
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_RA8875
#include "ra8875.h"
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_GC9A01
#include "GC9A01.h"
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_JD79653A
#include "jd79653a.h"
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_UC8151D
#include "uc8151d.h"
#endif

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/

/* Initialize display */
void disp_driver_init(void);

/* Display flush callback */
void disp_driver_flush(lv_display_t * drv, const lv_area_t * area, uint8_t * color_map);

/* Display rounder callback, used with monochrome dispays */
void disp_driver_rounder(lv_display_t * disp_drv, lv_area_t * area);
/* Display release https://github.com/lvgl/lvgl/blob/a36816ad768eda32e4714d94e91a1fa5785e0c15/src/drivers/nuttx/lv_nuttx_lcd.c#L209C13-L209C47 */
void disp_release_cb(lv_event_t * e);

/* Display set_px callback, used with monochrome dispays: Deprecated in LVGL>=9 */
void disp_driver_set_px(lv_display_t * disp_drv, uint8_t * buf, lv_coord_t buf_w, lv_coord_t x, lv_coord_t y,
    lv_color_t color, lv_opa_t opa);

/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*DISP_DRIVER_H*/
