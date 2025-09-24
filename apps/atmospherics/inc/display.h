#ifndef _ATMOS_DISPLAY_H_
#define _ATMOS_DISPLAY_H_

#include "zephyr/drivers/sensor.h"
#include <errno.h>
#include <stdint.h>
#include <zephyr/types.h>

typedef enum curr_display_page {
  PAGE_A = 0, PAGE_B, PAGE_C
} curr_display_page_t;

error_t init_display_device(void);
error_t init_framebuffer(void);
error_t printd(char *str, uint8_t row);
error_t set_display_choice(curr_display_page_t page);
error_t get_display_choice(curr_display_page_t* page);
error_t display_render_page(void);
//error_t display_page_a(const struct sensor_value *pm1_0, const struct sensor_value* pm0_5,
//               const struct sensor_value* tps);
//error_t display_page_b(const struct sensor_value *voc, const struct sensor_value* pm10,
//               const struct sensor_value *pm4_0, const struct sensor_value* pm2_5);
error_t display_page_a(void);
error_t display_page_b(void);
error_t display_page_c(void);
error_t display_clear(void);
#endif // !_ATMOS_DISPLAY_H_
