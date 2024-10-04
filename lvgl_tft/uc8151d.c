/**
    @file uc8151d.c
    @brief   GoodDisplay GDEW0154M10 DES e-paper display w/ UltraChip UC8151D
    @version 1.0
    @date    2020-08-28
    @author  Jackson Ming Hu <huming2207@gmail.com>


    @section LICENSE

    MIT License

    Copyright (c) 2020 Jackson Ming Hu

    Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
    to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute,
    sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
    IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
    TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */


#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <driver/gpio.h>
#include <esp_log.h>

#include "disp_spi.h"
#include "disp_driver.h"
#include "uc8151d.h"

#define TAG "lv_uc8151d"

#define PIN_DC              CONFIG_LV_DISP_PIN_DC
#define PIN_DC_BIT          ((1ULL << (uint8_t)(CONFIG_LV_DISP_PIN_DC)))
#define PIN_RST             CONFIG_LV_DISP_PIN_RST
#define PIN_RST_BIT         ((1ULL << (uint8_t)(CONFIG_LV_DISP_PIN_RST)))
#define PIN_BUSY            CONFIG_LV_DISP_PIN_BUSY
#define PIN_BUSY_BIT        ((1ULL << (uint8_t)(CONFIG_LV_DISP_PIN_BUSY)))
#define EVT_BUSY            (1UL << 0UL)
#define EPD_WIDTH           LV_HOR_RES
#define EPD_HEIGHT          LV_VER_RES
#define EPD_ROW_LEN         (EPD_HEIGHT / 8u)

#define BIT_SET(a, b)       ((a) |= (1U << (b)))
#define BIT_CLEAR(a, b)     ((a) &= ~(1U << (b)))

typedef struct
{
    uint8_t cmd;
    uint8_t data[3];
    size_t len;
} uc8151d_seq_t;

#define EPD_SEQ_LEN(x) ((sizeof(x) / sizeof(uc8151d_seq_t)))

static EventGroupHandle_t uc8151d_evts = NULL;
static uint8_t old_data[9600] = { 0 };

static void IRAM_ATTR uc8151d_busy_intr(void *arg)
{
    BaseType_t xResult;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xResult = xEventGroupSetBitsFromISR(uc8151d_evts, EVT_BUSY, &xHigherPriorityTaskWoken);
    if (xResult == pdPASS) {
        portYIELD_FROM_ISR();
    }
}

static void uc8151d_spi_send_cmd(uint8_t cmd)
{
    disp_wait_for_pending_transactions();
    gpio_set_level(PIN_DC, 0);     // DC = 0 for command
    disp_spi_send_data(&cmd, 1);
}

static void uc8151d_spi_send_data(uint8_t *data, size_t len)
{
    disp_wait_for_pending_transactions();
    gpio_set_level(PIN_DC, 1);  // DC = 1 for data
    disp_spi_send_data(data, len);
}

static void uc8151d_spi_send_data_byte(uint8_t data)
{
    disp_wait_for_pending_transactions();
    gpio_set_level(PIN_DC, 1);  // DC = 1 for data
    disp_spi_send_data(&data, 1);
}

static void uc8151d_spi_send_fb(uint8_t *data, size_t len)
{
    disp_wait_for_pending_transactions();
    gpio_set_level(PIN_DC, 1);   // DC = 1 for data
    disp_spi_send_colors(data, len);
}

static void uc8151d_spi_send_seq(const uc8151d_seq_t *seq, size_t len)
{
    ESP_LOGD(TAG, "Writing cmd/data sequence, count %u", len);

    if (!seq || len < 1) return;
    for (size_t cmd_idx = 0; cmd_idx < len; cmd_idx++) {
        uc8151d_spi_send_cmd(seq[cmd_idx].cmd);
        if (seq[cmd_idx].len > 0) {
            uc8151d_spi_send_data((uint8_t *) seq[cmd_idx].data, seq[cmd_idx].len);
        }
    }
}

static esp_err_t uc8151d_wait_busy(uint32_t timeout_ms)
{
    // uint32_t wait_ticks = (timeout_ms == 0 ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms));
    // EventBits_t bits = xEventGroupWaitBits(uc8151d_evts,
    //                                        EVT_BUSY, // Wait for busy bit
    //                                        pdTRUE, pdTRUE,       // Clear on exit, wait for all
    //                                        wait_ticks);         // Timeout

    // return ((bits & EVT_BUSY) != 0) ? ESP_OK : ESP_ERR_TIMEOUT;
    if(timeout_ms == 1) while(!gpio_get_level(PIN_BUSY));
    else while(gpio_get_level(PIN_BUSY));
    return ESP_OK;
}

static void uc8151d_sleep()
{
    // // Set VCOM to 0xf7
    // uc8151d_spi_send_cmd(0x50);
    // uc8151d_spi_send_data_byte(0xf7);

    // // Power off
    // uc8151d_spi_send_cmd(0x02);
    // uc8151d_wait_busy(0);

    // Go to sleep
    // uc8151d_spi_send_cmd(0x10);
    // uc8151d_spi_send_data_byte(0x01);
    // vTaskDelay(pdMS_TO_TICKS(100));

    //---------3.1寸
    uc8151d_spi_send_cmd(0x02);
    uc8151d_wait_busy(1);
    vTaskDelay(pdMS_TO_TICKS(100));
    uc8151d_spi_send_cmd(0x07);
    uc8151d_spi_send_data_byte(0xA5);

}


static void uc8151d_panel_init()
{
    // Hardware reset for 3 times - not sure why but it's from official demo code
    for (uint8_t cnt = 0; cnt < 1; cnt++) {
        gpio_set_level(PIN_RST, 0);
        vTaskDelay(pdMS_TO_TICKS(10)); // At least 10ms, leave 20ms for now just in case...
        gpio_set_level(PIN_RST, 1);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // uc8151d_wait_busy(0); //no fast refresh
    // uc8151d_spi_send_cmd(0x12);
    // uc8151d_wait_busy(0);

    // uc8151d_spi_send_cmd(0x21);
    // uc8151d_spi_send_data_byte(0x40);
    // uc8151d_spi_send_data_byte(0x00);
    // uc8151d_spi_send_cmd(0x3C);
    // uc8151d_spi_send_data_byte(0x05);
    // uc8151d_spi_send_cmd(0x1A);
    // uc8151d_spi_send_data_byte(0x6E);
    // uc8151d_spi_send_cmd(0x22);
    // uc8151d_spi_send_data_byte(0x91);
    // uc8151d_spi_send_cmd(0x20);
    // uc8151d_wait_busy(0);
    // uc8151d_spi_send_cmd(0x11);
    // uc8151d_spi_send_data_byte(0x01);

    // uc8151d_spi_send_cmd(0x01);
    // uc8151d_spi_send_data_byte((EPD_HEIGHT-1)%256);
    // uc8151d_spi_send_data_byte((EPD_HEIGHT-1)%256);
    // uc8151d_spi_send_data_byte(0x00);
    // uc8151d_spi_send_cmd(0x21);
    // uc8151d_spi_send_data_byte(0x40);
    // uc8151d_spi_send_data_byte(0x00);
    // uc8151d_spi_send_cmd(0x3C);
    // uc8151d_spi_send_data_byte(0x05);
    // uc8151d_spi_send_cmd(0x11);
    // uc8151d_spi_send_data_byte(0x01);

    // uc8151d_spi_send_cmd(0x44);
    // uc8151d_spi_send_data_byte(0x00);
    // uc8151d_spi_send_data_byte((EPD_WIDTH)/8-1);
    // uc8151d_spi_send_cmd(0x45);
    // uc8151d_spi_send_data_byte((EPD_HEIGHT-1)%256);
    // uc8151d_spi_send_data_byte((EPD_HEIGHT-1)/256);
    // uc8151d_spi_send_data_byte(0x00);
    // uc8151d_spi_send_data_byte(0x00);
    // uc8151d_spi_send_cmd(0x4E);
    // uc8151d_spi_send_data_byte(0x00);
    // uc8151d_spi_send_cmd(0x4F);
    // uc8151d_spi_send_data_byte((EPD_HEIGHT-1)%256);
    // uc8151d_spi_send_data_byte((EPD_HEIGHT-1)/256);
    // uc8151d_wait_busy(0);
    // Panel settings
//     uc8151d_spi_send_cmd(0x00);
// #if defined (CONFIG_LV_DISPLAY_ORIENTATION_PORTRAIT_INVERTED)
//     uc8151d_spi_send_data_byte(0x13);
// #elif defined (CONFIG_LV_DISPLAY_ORIENTATION_PORTRAIT)
//     uc8151d_spi_send_data_byte(0x1f);
// #endif

//     // VCOM & Data intervals
//     uc8151d_spi_send_cmd(0x50);
//     uc8151d_spi_send_data_byte(0x97);
//------------3.1寸

    uc8151d_spi_send_cmd(0x00);
    uc8151d_spi_send_data_byte(0x1F);
    uc8151d_spi_send_cmd(0x04);
    uc8151d_wait_busy(1);
    uc8151d_spi_send_cmd(0xE0);
    uc8151d_spi_send_data_byte(0x02);
    uc8151d_spi_send_cmd(0xE5);
    uc8151d_spi_send_data_byte(0x5A);

    printf("Panel init done\n");
}

static void uc8151d_full_update(uint8_t *buf)
{
    uc8151d_panel_init();
    
    uint8_t *buf_ptr = buf;
    

    // Fill old data
    // uc8151d_spi_send_cmd(0x24);
    // // for (size_t h_idx = 0; h_idx < EPD_HEIGHT; h_idx++) {
    // //     uc8151d_spi_send_data(old_data, EPD_ROW_LEN);
    // // }
    // // for (size_t h_idx = 0; h_idx < EPD_HEIGHT; h_idx++) {
    // //     uc8151d_spi_send_data(buf_ptr, EPD_ROW_LEN);
    // //     buf_ptr += EPD_ROW_LEN;
    // // }
    // for (size_t h_idx = 0; h_idx < 400*300/8; h_idx++) {
    //     uc8151d_spi_send_data_byte(buf_ptr[h_idx]);
    // }

    // buf_ptr = buf;
    // // Fill new data
    // uc8151d_spi_send_cmd(0x26);
    // // for (size_t h_idx = 0; h_idx < EPD_HEIGHT; h_idx++) {
    // //     uc8151d_spi_send_data(buf_ptr, EPD_ROW_LEN);
    // //     buf_ptr += EPD_ROW_LEN;
    // // }
    // for (size_t h_idx = 0; h_idx < 400*300/8; h_idx++) {
    //     uc8151d_spi_send_data_byte(buf_ptr[h_idx]);
    // }

    // // Issue refresh
    // uc8151d_spi_send_cmd(0x22);
    // uc8151d_spi_send_data_byte(0xC7);
    // uc8151d_spi_send_cmd(0x20);
    // printf("before wait busy\n");
    // uc8151d_wait_busy(0);
    // printf("after wait busy\n");

    // uc8151d_sleep();

    //-----------3.1寸
    
    uc8151d_spi_send_cmd(0x10);
    for(int i = 0;i<240*320/8;i++)
    {
        uc8151d_spi_send_data_byte(old_data[i]);
    }
    uc8151d_spi_send_cmd(0x13);
    for(int i = 0;i<240*320/8;i++)
    {
        uc8151d_spi_send_data_byte(buf_ptr[i]);
        // uc8151d_spi_send_data_byte(gImage_1[i]);
    }
    memcpy(old_data,buf_ptr,9600);
    // memcpy(old_data,gImage_1,9600);
    uc8151d_spi_send_cmd(0x12);
    ets_delay_us(200);
    uc8151d_wait_busy(1);
    uc8151d_spi_send_cmd(0x02);
    uc8151d_wait_busy(1);
    uc8151d_sleep();

}

static void uc8151d_fullwhite_update(uint8_t *buf)
{
    uc8151d_panel_init();
    
    uint8_t *buf_ptr = buf;
    

    // Fill old data
    // uc8151d_spi_send_cmd(0x24);
    // // for (size_t h_idx = 0; h_idx < EPD_HEIGHT; h_idx++) {
    // //     uc8151d_spi_send_data(old_data, EPD_ROW_LEN);
    // // }
    // // for (size_t h_idx = 0; h_idx < EPD_HEIGHT; h_idx++) {
    // //     uc8151d_spi_send_data(buf_ptr, EPD_ROW_LEN);
    // //     buf_ptr += EPD_ROW_LEN;
    // // }
    // for (size_t h_idx = 0; h_idx < 400*300/8; h_idx++) {
    //     uc8151d_spi_send_data_byte(buf_ptr[h_idx]);
    // }

    // buf_ptr = buf;
    // // Fill new data
    // uc8151d_spi_send_cmd(0x26);
    // // for (size_t h_idx = 0; h_idx < EPD_HEIGHT; h_idx++) {
    // //     uc8151d_spi_send_data(buf_ptr, EPD_ROW_LEN);
    // //     buf_ptr += EPD_ROW_LEN;
    // // }
    // for (size_t h_idx = 0; h_idx < 400*300/8; h_idx++) {
    //     uc8151d_spi_send_data_byte(buf_ptr[h_idx]);
    // }

    // // Issue refresh
    // uc8151d_spi_send_cmd(0x22);
    // uc8151d_spi_send_data_byte(0xC7);
    // uc8151d_spi_send_cmd(0x20);
    // printf("before wait busy\n");
    // uc8151d_wait_busy(0);
    // printf("after wait busy\n");

    // uc8151d_sleep();

    //-----------3.1寸
    
    uc8151d_spi_send_cmd(0x10);
    for(int i = 0;i<240*320/8;i++)
    {
        uc8151d_spi_send_data_byte(old_data[i]);
    }
    uc8151d_spi_send_cmd(0x13);
    for(int i = 0;i<240*320/8;i++)
    {
        // uc8151d_spi_send_data_byte(buf_ptr[i]);
        uc8151d_spi_send_data_byte(0xff);
        old_data[i] = 0xff;
    }
    // memcpy(old_data,buf_ptr,9600);
    // memcpy(old_data,gImage_1,9600);
    uc8151d_spi_send_cmd(0x12);
    ets_delay_us(200);
    uc8151d_wait_busy(1);
    uc8151d_spi_send_cmd(0x02);
    uc8151d_wait_busy(1);
    uc8151d_sleep();

}

void uc8151d_lv_fb_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    printf("Enter Flush\n");
    size_t len = ((area->x2 - area->x1 + 1) * (area->y2 - area->y1 + 1)) ;

    // ESP_LOGD(TAG, "x1: 0x%x, x2: 0x%x, y1: 0x%x, y2: 0x%x", area->x1, area->x2, area->y1, area->y2);
    printf("x1: %d, x2: %d, y1: %d, y2: %d", area->x1, area->x2, area->y1, area->y2);
    printf("Writing LVGL fb with len: %u", len);

    // uc8151d_fullwhite_update(NULL);
    // lv_disp_flush_ready(drv);
    // vTaskDelay(pdMS_TO_TICKS(1000));

    uint8_t *buf = (uint8_t *) color_map;
    uc8151d_full_update(buf);
    printf("Exit Full update\n");

    lv_disp_flush_ready(drv);
    printf("exit flush ready Flush\n");
    ESP_LOGD(TAG, "Ready");
}

void uc8151d_lv_set_fb_cb(lv_disp_drv_t *disp_drv, uint8_t *buf, lv_coord_t buf_w, lv_coord_t x, lv_coord_t y,
                           lv_color_t color, lv_opa_t opa)
{
    // uint16_t byte_index = (x >> 3u) + (y * EPD_ROW_LEN);
    // uint8_t bit_index = x & 0x07u;
    // printf("Setting at x: %u, y: %u\n", x, y);
    // if (color.full) {
    //     BIT_SET(buf[byte_index], 7 - bit_index);
    // } else {
    //     ESP_LOGD(TAG, "Clear at x: %u, y: %u", x, y);
    //     BIT_CLEAR(buf[byte_index], 7 - bit_index);
    // }
    // Calculate the index based on position

    // int16_t epd_color = 255;
    // if(color.full < 254) epd_color = 0;
    
    // drawPixel((int16_t)x, (int16_t)y, epd_color,buf);
    // uint32_t byte_index = x/8 + (y * buf_w)/8;
    // if(byte_index % )
    
    // // Assuming each pixel is one byte (8 bits) representing grayscale levels
    // uint8_t grayscale_value = color.full; // Full 8-bit grayscale

    // // Set the corresponding buffer value to the grayscale level
    // buf[byte_index] = grayscale_value;
    // printf("Setting at buf w: %u,\n", (int16_t)color.full);
     // Calculate the byte index based on (x, y) position
    uint32_t byte_index = (y * (buf_w / 8)) + (x / 8);
    uint8_t bit_index = 7-(x % 8); // Adjust for the correct bit in the byte

    // Assuming color.full gives you the grayscale value (0-255)
    uint8_t grayscale_value = color.full; // Full 8-bit grayscale

    // Set the corresponding buffer value based on grayscale
    if (grayscale_value > 127) {
        // Set the pixel to black
        BIT_SET(buf[byte_index], bit_index);
    } else {
        // Clear the pixel (white)
        BIT_CLEAR(buf[byte_index], bit_index);
    }
}

void uc8151d_lv_rounder_cb(lv_disp_drv_t *disp_drv, lv_area_t *area)
{
    // Always send full framebuffer if it's not in partial mode
    area->x1 = 0;
    area->y1 = 0;
    area->x2 = (EPD_WIDTH - 1);
    area->y2 = (EPD_HEIGHT - 1);
    // printf("Rounding\n");
}

void uc8151d_init()
{
    // Initialise event group
    uc8151d_evts = xEventGroupCreate();
    if (!uc8151d_evts) {
        ESP_LOGE(TAG, "Failed when initialising event group!");
        return;
    }

    // Setup output pins, output (PP)
    gpio_config_t out_io_conf = {
            .intr_type = GPIO_INTR_DISABLE,
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = PIN_DC_BIT | PIN_RST_BIT,
            .pull_down_en = 0,
            .pull_up_en = 0,
    };
    ESP_ERROR_CHECK(gpio_config(&out_io_conf));

    // Setup input pin, pull-up, input
    gpio_config_t in_io_conf = {
            .intr_type = GPIO_INTR_NEGEDGE,
            .mode = GPIO_MODE_INPUT,
            .pin_bit_mask = PIN_BUSY_BIT,
            .pull_down_en = 0,
            .pull_up_en = 1,
    };
    ESP_ERROR_CHECK(gpio_config(&in_io_conf));
    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_BUSY, uc8151d_busy_intr, (void *) PIN_BUSY);

    ESP_LOGI(TAG, "IO init finished");
    uc8151d_panel_init();
    // uc8151d_whiteScreen_init();
    // vTaskDelay(pdMS_TO_TICKS(2000));
    // uc8151d_fullwhite_update(NULL);
    // vTaskDelay(pdMS_TO_TICKS(2000));
    // uc8151d_full_update(NULL);
    // vTaskDelay(pdMS_TO_TICKS(2000));
    ESP_LOGI(TAG, "Panel initialised");
    

}
