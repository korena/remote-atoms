#ifndef _SCD41_SENSOR_H_
#define _SCD41_SENSOR_H_

#include <errno.h>
#include <zephyr/device.h>

struct scd41_data {
  uint16_t serial_number;
  uint16_t co2_concentration;
  uint32_t temperature;
  uint16_t relative_humidity;
};

error_t init_scd41_device(void);
error_t get_scd41_data(struct scd41_data*);
#endif // !_SCD41_SENSOR_H_

