#ifndef _BME_SENSOR_H_
#define _BME_SENSOR_H_

#include "zephyr/drivers/sensor.h"
#include <zephyr/device.h>
#include <zephyr/kernel.h>

struct bme280_data {
  struct sensor_value temp;
  struct sensor_value press;
  struct sensor_value humidity;
};

error_t init_bme280_device(void);
error_t get_bme280_data(struct bme280_data*);

#endif // !_BME_SENSOR_H_
