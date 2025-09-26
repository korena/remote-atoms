#ifndef _SPS30_SENSOR_H_
#define _SPS30_SENSOR_H_

#include "zephyr/drivers/sensor.h"
#include <errno.h>
#include <zephyr/device.h>


struct sps30_data {
  struct sensor_value mc_1p0;
  struct sensor_value mc_2p5;
  struct sensor_value mc_4p0;
  struct sensor_value mc_10p0;
  struct sensor_value nc_0p5;
  struct sensor_value nc_1p0;
  struct sensor_value nc_2p5;
  struct sensor_value nc_4p0;
  struct sensor_value nc_10p0;
  struct sensor_value typical_particle_size;
};

error_t init_sps30_device(void);
error_t get_sps30_data(struct sps30_data*);

#endif // !_SPS30_SENSOR_H_
