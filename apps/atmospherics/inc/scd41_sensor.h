#ifndef _SCD41_SENSOR_H_
#define _SCD41_SENSOR_H_

#include "zephyr/drivers/sensor.h"
#include <errno.h>
#include <zephyr/device.h>

struct scd41_data {
  struct sensor_value co2_concentration;
  struct sensor_value temperature;
  struct sensor_value relative_humidity;
};

error_t init_scd41_device(void);
error_t update_scd41_pressure(const struct sensor_value* pressure);
error_t get_scd41_data(struct scd41_data*);
#endif // !_SCD41_SENSOR_H_

