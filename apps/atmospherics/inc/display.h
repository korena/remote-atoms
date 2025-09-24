#ifndef _ATMOS_DISPLAY_H_
#define _ATMOS_DISPLAY_H_

#include "zephyr/drivers/sensor.h"
#include <errno.h>
#include <stdint.h>
#include <zephyr/types.h>

error_t init_display_device(void);
error_t init_framebuffer(void);
error_t printd(char *str, uint8_t row);
error_t page_c(const struct sensor_value *temp, const struct sensor_value *hum,
               const struct sensor_value *press, const struct sensor_value* co2);
error_t page_b(const struct sensor_value *voc, const struct sensor_value* pm10,
               const struct sensor_value *pm4_0, const struct sensor_value* pm2_5);
error_t page_a(const struct sensor_value *pm1_0, const struct sensor_value* pm0_5,
               const struct sensor_value* tps);
error_t display_clear(void);
#endif // !_ATMOS_DISPLAY_H_
